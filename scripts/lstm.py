import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import matplotlib.pyplot as plt
import pandas as pd
from dataclasses import dataclass
from torch.utils.data import DataLoader, Dataset
from racetrack_utilities.racetrack_utilities import RacetrackUtilities


rutil = RacetrackUtilities("/sim_ws/src/lidar_processing/scripts/Spielberg_map.csv")

STALE_POSITION_EPS = 1e-6

# IMPORTANT: unwrap the data's Frenet s with the SAME period the geometry uses.
# opp_s / ego_s in the logs must share the arc-length scale of this map, or the
# fan indexing below will point at the wrong track positions. Use the spline's
# own arclength rather than a hardcoded 360.0.
TRACK_LENGTH = float(rutil.arclength)

# ---- Fan / context hyperparameters ----
FAN_RESOLUTION = 1000      # stations over the whole track in the precomputed fan
NUM_OFFSETS = 5            # lines in the fan
SEG_POINTS = 20            # stations per extracted segment (must match the CNN)
RELATIVE_S = True          # only makes the FAN s relative, model s stays global


def unwrap_s_values(values, track_length=TRACK_LENGTH):
    """Convert wrapped Frenet s values into one continuous sequence."""
    values = np.asarray(values, dtype=np.float64)
    angles = values * (2.0 * np.pi / track_length)
    return np.unwrap(angles) * (track_length / (2.0 * np.pi))


# -----------------------------
# DATA STRUCTURES
# -----------------------------

@dataclass
class TrajectoryPoint:
    t: float
    opp_s: float
    opp_d: float
    ego_s: float
    ego_d: float
    ego_v: float


# -----------------------------
# CSV Parsing
# -----------------------------

def parse_csv_to_points(csv_path):
    df = pd.read_csv(csv_path)

    df["opp_s"] = unwrap_s_values(df["opp_s"].to_numpy())
    df["ego_s"] = unwrap_s_values(df["ego_s"].to_numpy())

    points = []
    for _, row in df.iterrows():
        points.append(TrajectoryPoint(
            t=row["timestamp"],
            opp_s=row["opp_s"], opp_d=row["opp_d"],
            ego_s=row["ego_s"], ego_d=row["ego_d"], ego_v=row["ego_vel"],
        ))
    return points


# -----------------------------
# BATCHED POLYLINE SEGMENT EXTRACTION
# -----------------------------

def get_polyline_segments(fan, arclength, s_start, s_end,
                          num_points=SEG_POINTS, relative_s=RELATIVE_S):
    """Extract one fixed-length polyline segment per batch element by resampling
    the precomputed whole-track fan over [s_start, s_end].

    Parameters
    ----------
    fan : (P, N, 3) torch tensor on the target device
        Columns [s_abs, d, offset_curvature] from RacetrackUtilities.precompute_fan.
    arclength : float
        Period of the fan (metres); the fan spans one lap [0, arclength).
    s_start, s_end : (B,) torch tensors
        Unwrapped Frenet s (metres) at the start and end of each sample's window.
    num_points : int
        Stations to resample to (fixed, so the CNN sees a constant length).
    relative_s : bool
        If True the FAN s channel is replaced by Delta s from s_start. This does
        not change the global Frenet s values given to or predicted by the LSTM.

    Returns
    -------
    (B, P, num_points, 3) torch tensor.
    """
    P, N, _ = fan.shape
    device = fan.device
    s_start = s_start.to(device=device, dtype=fan.dtype)
    s_end = s_end.to(device=device, dtype=fan.dtype)

    t = torch.linspace(0.0, 1.0, num_points, device=device, dtype=fan.dtype)       # (L,)
    query_s = s_start[:, None] + (s_end - s_start)[:, None] * t[None, :]          # (B, L)

    s_mod = torch.remainder(query_s, arclength)                                    # wrap into one lap
    idx = (torch.round(s_mod / arclength * N).long()) % N                          # (B, L)

    seg = fan[:, idx, :]                          # (P, B, L, 3)
    seg = seg.permute(1, 0, 2, 3).contiguous()    # (B, P, L, 3)

    if relative_s:
        rel = query_s - s_start[:, None]                          # (B, L)
        seg[..., 0] = rel[:, None, :].expand(-1, P, -1)

    return seg


# -----------------------------
# DATASET
# -----------------------------

class TrajectoryDataset(Dataset):
    def __init__(self, points, history_len=15, future_len=10):
        self.points = points
        self.history_len = history_len
        self.future_len = future_len
        self.samples = self.create_samples()

    def create_samples(self):
        samples = []
        max_start = len(self.points) - self.history_len - self.future_len + 1

        for start_idx in range(max_start):
            history = self.points[start_idx: start_idx + self.history_len]
            future = self.points[start_idx + self.history_len: start_idx + self.history_len + self.future_len]

            x = []
            for i, point in enumerate(history):
                dt = 0.0 if i == 0 else point.t - history[i - 1].t
                
                x.append([point.opp_s, point.opp_d, point.ego_s, point.ego_d, point.ego_v, dt])

            # Keep the FUTURE GLOBAL state. During recursive training:
            #   - future opp_s / opp_d provide the true next global position
            #   - the model itself predicts a one-step opponent delta
            #   - future ego_s / ego_d / ego_v are still real inputs from ROS
            y = []

            previous_t = history[-1].t

            for point in future:
                dt = point.t - previous_t

                y.append([
                    point.opp_s,
                    point.opp_d,
                    point.ego_s,
                    point.ego_d,
                    point.ego_v,
                    dt,
                ])

                previous_t = point.t

            samples.append((
                np.array(x, dtype=np.float32),
                np.array(y, dtype=np.float32),
            ))
        return samples

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        x, y = self.samples[idx]
        return torch.tensor(x), torch.tensor(y)


# -----------------------------
# MODEL
# -----------------------------

class LSTMModel(nn.Module):
    def __init__(self, n_hidden=51, n_context=32, num_offsets=NUM_OFFSETS):
        super().__init__()
        self.n_hidden = n_hidden
        self.n_context = n_context
        self.num_offsets = num_offsets

        # Per-polyline encoder: one polyline is (3 channels, 20 stations).
        # Shared across all offsets (offset axis rides along in the batch dim).
        per_poly = 30
        self.polyline_encoder = nn.Sequential(
            nn.Conv1d(3, 6, 3, 2, padding=1, padding_mode='replicate'),   # (6, 10)
            nn.ReLU(),
            nn.Conv1d(6, 12, 3, 2, padding=1, padding_mode='replicate'),  # (12, 5)
            nn.ReLU(),
            nn.Flatten(),
            nn.Linear(60, per_poly),
            nn.ReLU(),
        )
        # Combine the num_offsets per-line embeddings into one context vector.
        self.context_head = nn.Sequential(
            nn.Linear(num_offsets * per_poly, n_context),
            nn.ReLU(),
        )

        self.encoder_lstm1 = nn.LSTMCell(input_size=6 + n_context, hidden_size=n_hidden)
        self.encoder_lstm2 = nn.LSTMCell(input_size=n_hidden, hidden_size=n_hidden)
        self.linear = nn.Linear(n_hidden, 2)

    def encode_polylines(self, segments):
        """segments: (B, P, L, 3) -> context: (B, n_context)."""
        B, P, L, F = segments.shape
        seg = segments.permute(0, 1, 3, 2).reshape(B * P, F, L)   # (B*P, 3, L)
        emb = self.polyline_encoder(seg)                          # (B*P, per_poly)
        emb = emb.reshape(B, P * emb.shape[-1])                   # (B, P*per_poly)
        return self.context_head(emb)                             # (B, n_context)

    def forward(self, x, polyline_segments):
        """x: (B, history_len, 5); returns predicted [delta_s, delta_d]."""
        batch_size = x.size(0)

        context = self.encode_polylines(polyline_segments)   # (B, n_context)

        h_t = x.new_zeros(batch_size, self.n_hidden)
        c_t = x.new_zeros(batch_size, self.n_hidden)
        h_t2 = x.new_zeros(batch_size, self.n_hidden)
        c_t2 = x.new_zeros(batch_size, self.n_hidden)

        # Encode the entire current history every time the model is called.
        for input_t in x.unbind(dim=1):
            enc_in = torch.cat([input_t, context], dim=1)        # (B, 5 + n_context)
            h_t, c_t = self.encoder_lstm1(enc_in, (h_t, c_t))
            h_t2, c_t2 = self.encoder_lstm2(h_t, (h_t2, c_t2))

        # ONE-STEP opponent displacement.
        # Inputs stay in global Frenet coordinates; only the output is a delta.
        return self.linear(h_t2)                                 # (B, 2) [delta_s, delta_d]


# -----------------------------
# AUTOMATIC ROLLOUT CURRICULUM
# -----------------------------

def get_rollout_len(epoch):
    """Automatically increase recursive training difficulty as training progresses."""
    if epoch < 20:
        return 1
    elif epoch < 40:
        return 3
    elif epoch < 60:
        return 5
    else:
        return 10


# -----------------------------
# RECURSIVE ROLLOUT
# -----------------------------

def recursive_rollout(model, x, y, fan, arclength):
    """Repeatedly predict one-step opponent deltas, convert them back to global
    positions, and feed those global predictions into the next history window."""
    history = x
    outputs = []

    for step in range(y.shape[1]):
        # Rebuild the racetrack context from the CURRENT history. The context
        # extends from the oldest opponent s to one estimated step ahead.
        s_start = history[:, 0, 0]
        opp_last_delta_s = history[:, -1, 0] - history[:, -2, 0]
        s_end = history[:, -1, 0] + opp_last_delta_s
        segments = get_polyline_segments(fan, arclength, s_start, s_end)

        # The network predicts how far the opponent moves from the CURRENT
        # global opponent position in the history.
        pred_delta = model(history, segments)             # [delta_s, delta_d]

        current_opp = history[:, -1, 0:2]
        pred_global = current_opp + pred_delta             # global [opp_s, opp_d]
        outputs.append(pred_global)

        # Opponent position is predicted, but ego state is still ground truth
        # because ROS will continue providing ego_s, ego_d, and ego_v.
        next_ego_and_dt = y[:, step, 2:6]

        next_input = torch.cat([
            pred_global,
            next_ego_and_dt,
        ], dim=1)

        # Drop the oldest point and append the new GLOBAL predicted position
        # for the next LSTM call.
        history = torch.cat([history[:, 1:, :], next_input.unsqueeze(1)], dim=1)

    return torch.stack(outputs, dim=1)                    # (B, future_len, 2)


# -----------------------------
# TRAINING AND TESTING
# -----------------------------

if __name__ == "__main__":
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")
    if device.type == "cuda":
        print(f"GPU: {torch.cuda.get_device_name(0)}")

    # Precompute the whole-track fan ONCE and move it to the device.
    fan_np, d_values = rutil.precompute_fan(
        num_points_total=FAN_RESOLUTION, num_offsets=NUM_OFFSETS
    )
    fan_t = torch.tensor(fan_np, dtype=torch.float32, device=device)   # (P, N, 3)
    arclength = float(rutil.arclength)
    print(f"Fan: {tuple(fan_t.shape)}  offsets={np.round(d_values, 3)}  arclength={arclength:.2f}")

    csv_path = "fixed_data/session_1_data_fixed.csv"
    val_csv_path = "fixed_data/session_3_data_fixed.csv"
    points = parse_csv_to_points(csv_path)
    val_points = parse_csv_to_points(val_csv_path)

    # Keep 10 future ground-truth states available. The training curriculum below
    # automatically decides how many recursive steps to use at each epoch.
    dataset = TrajectoryDataset(points, history_len=15, future_len=10)
    val_dataset = TrajectoryDataset(val_points, history_len=15, future_len=10)

    use_cuda = device.type == "cuda"
    dataloader = DataLoader(dataset, batch_size=128, shuffle=True, pin_memory=use_cuda)
    val_dataloader = DataLoader(val_dataset, batch_size=128, shuffle=False, pin_memory=use_cuda)

    # Global inputs -> one-step opponent delta output.
    # Each predicted delta is immediately added to the current global opponent
    # position before being fed back into the recursive history.
    model = LSTMModel().to(device)
    criterion = nn.MSELoss()
    optimizer = optim.Adam(model.parameters(), lr=0.001)
    scheduler = optim.lr_scheduler.ReduceLROnPlateau(
        optimizer, mode="min", factor=0.5, patience=5, min_lr=1e-6
    )
    n_steps = 100
    best_val_loss = float('inf')
    save_path = "singleposbestmodel.pt"
    previous_rollout_len = None

    for i in range(n_steps):
        rollout_len = get_rollout_len(i)

        # A new rollout length is a new training stage. Reset the stage-specific
        # best validation loss and LR scheduler, but KEEP the trained model and
        # optimizer weights/momentum.
        if rollout_len != previous_rollout_len:
            print(f"\n=== Starting {rollout_len}-step rollout stage at epoch {i} ===")
            best_val_loss = float('inf')
            scheduler = optim.lr_scheduler.ReduceLROnPlateau(
                optimizer, mode="min", factor=0.5, patience=5, min_lr=1e-6
            )
            previous_rollout_len = rollout_len
        model.train()
        total_loss = 0.0

        for x_batch, y_batch in dataloader:
            x_batch = x_batch.to(device, non_blocking=True)
            y_batch = y_batch.to(device, non_blocking=True)

            optimizer.zero_grad()

            # Automatically use 1, 3, 5, or 10 recursive predictions depending
            # on the current training stage. The model predicts one-step deltas;
            # recursive_rollout converts each delta back to a global position.
            rollout_y = y_batch[:, :rollout_len, :]
            pred = recursive_rollout(model, x_batch, rollout_y, fan_t, arclength)
            target = rollout_y[:, :, 0:2]
            loss = criterion(pred, target)

            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=5.0)
            optimizer.step()
            total_loss += loss.item() * x_batch.size(0)

        total_loss /= len(dataset)
        print(f"Training Loss for Epoch {i} ({rollout_len}-step rollout): {total_loss}")

        model.eval()
        val_loss = 0.0
        one_step_val_loss = 0.0
        with torch.no_grad():
            for x_batch, y_batch in val_dataloader:
                x_batch = x_batch.to(device, non_blocking=True)
                y_batch = y_batch.to(device, non_blocking=True)

                # Current curriculum-stage recursive validation.
                rollout_y = y_batch[:, :rollout_len, :]
                pred = recursive_rollout(model, x_batch, rollout_y, fan_t, arclength)
                target = rollout_y[:, :, 0:2]
                val_loss += criterion(pred, target).item() * x_batch.size(0)

                # Also track the immediate next-position error every epoch so we
                # can see whether basic one-step accuracy is being preserved.
                one_step_pred = pred[:, 0, :]
                one_step_target = y_batch[:, 0, 0:2]
                one_step_val_loss += criterion(
                    one_step_pred, one_step_target
                ).item() * x_batch.size(0)

        val_loss /= len(val_dataset)
        one_step_val_loss /= len(val_dataset)

        print(
            f"Validation Loss for Epoch {i} "
            f"({rollout_len}-step rollout): {val_loss}"
        )
        print(f"One-step Validation Loss: {one_step_val_loss}")

        scheduler.step(val_loss)
        print(f"Learning rate: {optimizer.param_groups[0]['lr']:.8f}")
        if val_loss < best_val_loss:
            best_val_loss = val_loss
            torch.save(model.state_dict(), save_path)
            print(f"New best model saved (val loss {val_loss: .4f})")

    # ---- One-step constant-position baseline ----
    baseline_loss = 0.0
    with torch.no_grad():
        for x_batch, y_batch in val_dataloader:
            x_batch = x_batch.to(device, non_blocking=True)
            y_batch = y_batch.to(device, non_blocking=True)
            target = y_batch[:, 0, 0:2]

            baseline_pred = x_batch[:, -1, 0:2]
            baseline_loss += criterion(baseline_pred, target).item() * y_batch.size(0)

    baseline_loss /= len(val_dataset)
    print(f"Constant-position baseline: {baseline_loss}")

    # ---- One-step constant-velocity baseline ----
    constant_velocity_loss = 0.0
    with torch.no_grad():
        for x_batch, y_batch in val_dataloader:
            x_batch = x_batch.to(device, non_blocking=True)
            y_batch = y_batch.to(device, non_blocking=True)
            target = y_batch[:, 0, 0:2]

            last_position = x_batch[:, -1, 0:2]
            last_delta = x_batch[:, -1, 0:2] - x_batch[:, -2, 0:2]
            baseline_pred = last_position + last_delta

            constant_velocity_loss += criterion(baseline_pred, target).item() * x_batch.size(0)

    constant_velocity_loss /= len(val_dataset)
    print(f"Constant-velocity baseline: {constant_velocity_loss}")

    # ---- Per-axis error metrics (final 10-step recursive rollout) ----
    with torch.no_grad():
        total_s_sq = total_d_sq = total_s_abs = total_d_abs = 0.0
        total_values = 0

        for x_batch, y_batch in val_dataloader:
            x_batch = x_batch.to(device, non_blocking=True)
            y_batch = y_batch.to(device, non_blocking=True)

            pred = recursive_rollout(model, x_batch, y_batch, fan_t, arclength)
            target = y_batch[:, :, 0:2]

            s_error = pred[:, :, 0] - target[:, :, 0]
            d_error = pred[:, :, 1] - target[:, :, 1]
            total_s_sq += torch.sum(s_error ** 2).item()
            total_d_sq += torch.sum(d_error ** 2).item()
            total_s_abs += torch.sum(torch.abs(s_error)).item()
            total_d_abs += torch.sum(torch.abs(d_error)).item()
            total_values += target.shape[0] * target.shape[1]

        print(f"Validation s RMSE: {np.sqrt(total_s_sq / total_values):.6f}")
        print(f"Validation d RMSE: {np.sqrt(total_d_sq / total_values):.6f}")
        print(f"Validation s MAE:  {total_s_abs / total_values:.6f}")
        print(f"Validation d MAE:  {total_d_abs / total_values:.6f}")