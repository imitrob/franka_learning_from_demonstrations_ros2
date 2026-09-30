import dash
from dash import dcc, html, Input, Output, State, no_update, clientside_callback
import plotly.graph_objs as go
import numpy as np
import os
from PIL import Image
import base64
import io
import plotly.io as pio  # uses Kaleido for image export
from functools import lru_cache

import socket, uuid, os, io, base64
from IPython.display import display
from IPython.display import Image as IPyImage

TRAJECTORYPLOT_MARGINS = dict(l=90, r=90, b=90, t=70)

# Configuration
import trajectory_data, object_localization
TRAJECTORIES_DIR = f"/home/imitlearn/petr_sandbox/saw_ws/src/trajectory_data/trajectories/quantitative_study"
TRAJECTORIES_DIR = f"{trajectory_data.package_path}/trajectories"
CFG_DIR = f"/home/imitlearn/petr_sandbox/saw_ws/src/ILeSiA/franka_risk_aware_learning_from_demonstrations/object_localization/cfg"
CFG_DIR = f"{object_localization.package_path}/cfg"

from pathlib import Path
Path(CFG_DIR).mkdir(parents=True, exist_ok=True)

app = dash.Dash(__name__)

app.layout = html.Div([
    html.H1("Robot Skill Visualization Dashboard"),
    html.Div([
        dcc.Dropdown(
            id='skill-dropdown',
            options=[{'label': f, 'value': f} 
                    for f in os.listdir(TRAJECTORIES_DIR) 
                    if f.endswith('.npz')],
            placeholder="Select Skill File"
        ),
        dcc.Dropdown(
            id='template-dropdown',
            options=[{'label': d, 'value': d} 
                    for d in os.listdir(CFG_DIR) 
                    if os.path.isdir(os.path.join(CFG_DIR, d))],
            placeholder="Select Template"
        )
    ], style={'margin': '20px', 'width': '500px'}),
    
    html.Div([
        dcc.Graph(
            id='3d-trajectory',
            style={'width': '49%', 'display': 'inline-block'}
        ),
        dcc.Graph(
            id='gripper-plot',
            style={'width': '49%', 'display': 'inline-block'}
        ),
        html.Div([
            html.Button(
                "Download SVG (client-side)", id="btn-download-svg-client",
                style={"padding":"8px 12px","borderRadius":"8px","border":"1px solid #ddd",
                    "background":"#fafafa","cursor":"pointer","fontWeight":600}
            ),
            html.Div(id="noop", style={"display":"none"})
        ], style={"margin":"12px 0"})
    ]),
    
    html.Div([
        html.Div([
            html.H3("Trajectory Point View"),
            html.Img(id='trajectory-image', style={'height': '300px'})
        ], style={'width': '32%', 'display': 'inline-block'}),
        
        html.Div([
            html.H3("Template Full Image"),
            html.Img(id='template-full-image', style={'height': '300px'})
        ], style={'width': '32%', 'display': 'inline-block'}),
        
        html.Div([
            html.H3("Template Cropped"),
            html.Img(id='template-cropped-image', style={'height': '300px'})
        ], style={'width': '32%', 'display': 'inline-block'})
    ])
])

def load_skill_data(skill_file):
    """Load skill data from .npz file"""
    data = np.load(os.path.join(TRAJECTORIES_DIR, skill_file))
    if "tag" in data.keys():
        tag = str(data['tag.npy'])
    else:
        tag = "no tag"
    return {
        'traj': data['traj.npy'],
        'grip': data['grip.npy'],
        'images': data['img.npy'],
        'tag': tag,
    }

def load_template(template_name):
    """Load template images"""
    template_dir = os.path.join(CFG_DIR, template_name)
    return {
        'full': Image.open(os.path.join(template_dir, 'full_image.png')),
        'cropped': Image.open(os.path.join(template_dir, 'template.png'))
    }

def numpy_to_base64_png(img_array):
    """Convert numpy array image to base64"""
    img = Image.fromarray(img_array.astype('uint8'))
    buffered = io.BytesIO()
    img.save(buffered, format="PNG")
    return f"data:image/png;base64,{base64.b64encode(buffered.getvalue()).decode()}"


def numpy_to_base64(img_array):
    """Convert numpy array image to base64"""
    # Handle grayscale images (h, w) or (n, h, w)
    if len(img_array.shape) == 2:
        img = Image.fromarray(img_array.astype('uint8'), mode='L')
    elif len(img_array.shape) == 3:
        img = Image.fromarray(img_array.astype('uint8'), mode='L')
    else:
        raise ValueError("Unsupported image shape")
    
    buffered = io.BytesIO()
    img.save(buffered, format="PNG")
    return f"data:image/png;base64,{base64.b64encode(buffered.getvalue()).decode()}"


# Registered on THIS module's app (not the global dash registry), otherwise
# any dashboard that merely imports this module inherits the callback and
# complains that btn-download-svg-client is not in its layout.
app.clientside_callback(
    """
    async function(n) {
      if (!n) { return window.dash_clientside.no_update; }
      const container = document.getElementById('3d-trajectory');
      if (!container) { return null; }
      const gd = container.getElementsByClassName('js-plotly-plot')[0];
      if (!gd) { return null; }
      try {
        // Uses current camera/view exactly as seen on screen
        const url = await Plotly.toImage(gd, {format:'svg', width:600, height:600});
        const a = document.createElement('a');
        a.href = url;
        a.download = 'trajectory_3d.svg';
        document.body.appendChild(a);
        a.click();
        a.remove();
      } catch (e) {
        console.error('SVG export failed:', e);
        alert('SVG export failed: ' + e);
      }
      return null;
    }
    """,
    Output("noop", "children"),
    Input("btn-download-svg-client", "n_clicks")
)

@app.callback(
    [Output('3d-trajectory', 'figure'),
     Output('gripper-plot', 'figure')],
    [Input('skill-dropdown', 'value')]
)
def update_skill_visualization(skill_file):
    if not skill_file:
        return go.Figure(), go.Figure()
    
    data = load_skill_data(skill_file)
    traj = data['traj']
    grip = data['grip'].squeeze()
    
    # Prepare point indices for customdata
    point_indices = np.arange(traj.shape[1])
    
    # 3D Trajectory Plot
    trajectory_fig = go.Figure(
        data=[go.Scatter3d(
            x=traj[0,:], 
            y=traj[1,:], 
            z=traj[2,:],
            mode='markers+lines',
            marker=dict(
                size=4,
                color=point_indices,
                colorscale='Viridis',
                colorbar=dict(title='Point Index')
            ),
            line=dict(color='royalblue', width=2),
            customdata=point_indices,
            hovertemplate='<b>Point %{customdata}</b><br>' +
                        'X: %{x:.2f}<br>Y: %{y:.2f}<br>Z: %{z:.2f}<extra></extra>'
        )]
    )
    trajectory_fig.update_layout(
        title='End-Effector Trajectory',
        scene=dict(
            aspectmode='data',
            xaxis_title='X',
            yaxis_title='Y',
            zaxis_title='Z'
        ),
        # more breathing room so axis titles stay inside the export
        margin=TRAJECTORYPLOT_MARGINS,
    )
    
    # Gripper State Plot
    gripper_fig = go.Figure(
        data=[go.Scatter(
            x=point_indices,
            y=grip,
            mode='lines+markers',
            line=dict(color='green'),
            hovertemplate='Point: %{x}<br>Value: %{y:.2f}<extra></extra>'
        )]
    )
    gripper_fig.update_layout(
        title='Gripper State',
        xaxis_title='Point Index',
        yaxis_title='Gripper opened [m], 0 is closed',
        margin=dict(l=0, r=0, b=0, t=40)
    )
    
    return trajectory_fig, gripper_fig

@app.callback(
    [Output('template-full-image', 'src'),
     Output('template-cropped-image', 'src')],
    [Input('template-dropdown', 'value')]
)
def update_template_images(template_name):
    if not template_name:
        return None, None
    
    template = load_template(template_name)
    full = numpy_to_base64_png(np.array(template['full']))
    cropped = numpy_to_base64_png(np.array(template['cropped']))
    return full, cropped

@app.callback(
    Output('trajectory-image', 'src'),
    [Input('3d-trajectory', 'clickData')],
    [State('skill-dropdown', 'value')]
)
def update_clicked_image(clickData, skill_file):
    if not clickData or not skill_file:
        return None
    
    try:
        point_idx = clickData['points'][0]['pointNumber']
        data = load_skill_data(skill_file)
        img_array = data['images'][point_idx]
        
        # Handle grayscale image (h, w) or (n, h, w)
        if len(img_array.shape) == 3:
            img_array = img_array[0]  # Take first frame if multiple
        
        return numpy_to_base64(img_array)
    except Exception as e:
        print(f"Error updating image: {e}")
        return None

TRAJ_COLORS = ['royalblue', 'darkorange', 'seagreen', 'crimson', 'purple', 'teal']


GRIPPER_OPEN_THRESHOLD = 0.04  # [m] gripper values >= this count as open

CURRENT_FRAME_TRACE = 'current frame'  # the marker a viewer moves, last trace


def gripper_glyph(point, is_open: bool, size: float):
    """A little gripper drawn at one waypoint, as (x, y, z) polylines.

    Scatter3d markers only come in Plotly's fixed symbol set and a 3D scene
    takes no images, so the gripper is line segments: a stem, a crossbar and
    two jaws whose gap is the whole point -- wide when the gripper is open,
    nearly shut when it is closed. Axis-aligned; the recorded `ori` would let
    it follow the wrist, at the cost of a quaternion rotation per frame."""
    cx, cy, cz = (float(c) for c in point)
    w = size * (0.55 if is_open else 0.14)   # half the jaw gap
    top, bar, tip = cz + size, cz + 0.35 * size, cz - 0.1 * size
    return (
        [cx, cx, None, cx - w, cx + w, None, cx - w, cx - w, None, cx + w, cx + w],
        [cy, cy, None, cy,     cy,     None, cy,     cy,     None, cy,     cy],
        [top, bar, None, bar,  bar,    None, bar,    tip,    None, bar,    tip],
    )


def _resolve_skill_path(skill_file):
    return skill_file if (os.path.isabs(skill_file) or os.path.sep in skill_file) \
        else os.path.join(TRAJECTORIES_DIR, skill_file)


@lru_cache(maxsize=32)
def load_traj(skill_file):
    """Load only the 'traj' member of a skill npz (fast: the heavy image
    array is never decompressed). Cached: a viewer re-reads it on every mouse
    move to place its marker. Treat the result as read-only."""
    npz = np.load(_resolve_skill_path(skill_file))
    return npz['traj'] if 'traj' in npz.files else npz['traj.npy']


@lru_cache(maxsize=6)
def load_images(skill_file):
    """The camera frames recorded alongside the trajectory, one per waypoint.

    Cached: the array is tens of MB and a viewer asks for one frame at a time
    while the mouse moves along the trajectory."""
    npz = np.load(_resolve_skill_path(skill_file))
    return npz['img'] if 'img' in npz.files else npz['img.npy']


def frame_data_uri(skill_file, index: int, quality: int = 72):
    """One recorded camera frame as a JPEG data URI (a few kB, ~2 ms), or None
    when the recording has no images or the index is out of range."""
    try:
        frames = load_images(skill_file)
    except Exception as e:  # noqa: BLE001 -- recording without an img member
        print(f"No camera frames in {skill_file}: {e}")
        return None
    if index < 0 or index >= len(frames):
        return None
    frame = frames[index]
    if frame.ndim == 3:  # (n, h, w): a single frame was stored wrapped
        frame = frame[0]
    buffered = io.BytesIO()
    Image.fromarray(frame.astype('uint8')).save(buffered, format="JPEG", quality=quality)
    return f"data:image/jpeg;base64,{base64.b64encode(buffered.getvalue()).decode()}"


@lru_cache(maxsize=32)
def load_grip(skill_file):
    """Load only the 'grip' member of a skill npz. Cached, read-only."""
    npz = np.load(_resolve_skill_path(skill_file))
    grip = npz['grip'] if 'grip' in npz.files else npz['grip.npy']
    return np.asarray(grip, dtype=float).squeeze()


def gripper_change_indices(grip, open_threshold=GRIPPER_OPEN_THRESHOLD):
    """Waypoint indices where the gripper switches between open and closed."""
    is_open = np.asarray(grip).squeeze() >= open_threshold
    return np.where(np.diff(is_open.astype(int)) != 0)[0] + 1


def trajectories_fig(named_trajs: dict, named_grips: dict = None):
    """Full End-Effector Trajectory view for one or more recordings overlaid
    in one 3D scene (e.g. the put1_* and put2_* parts of a double-object
    skill, or all recordings of an action). Keys are legend labels.
    When named_grips is given, the waypoints where the gripper state changes
    (open <-> close) are marked with red diamonds."""
    fig = go.Figure()
    change_pts = []  # (x, y, z, label, index, state)
    for i, (label, traj) in enumerate(named_trajs.items()):
        idx = np.arange(traj.shape[1])
        fig.add_trace(go.Scatter3d(
            x=traj[0, :], y=traj[1, :], z=traj[2, :],
            mode='markers+lines',
            marker=dict(size=3, color=idx, colorscale='Viridis', showscale=False),
            line=dict(color=TRAJ_COLORS[i % len(TRAJ_COLORS)], width=3),
            name=label,
            customdata=idx,
            # 'none', not 'skip': no tooltip box (it lands right on top of the
            # gripper glyph drawn at the hovered waypoint) but hover events
            # still fire, which is what drives the camera view.
            hoverinfo='none',
        ))
        grip = (named_grips or {}).get(label)
        if grip is not None:
            n = min(traj.shape[1], len(np.atleast_1d(grip)))
            for t in gripper_change_indices(grip[:n]):
                if t < n:
                    state = "opens" if grip[t] >= GRIPPER_OPEN_THRESHOLD else "closes"
                    change_pts.append((traj[0, t], traj[1, t], traj[2, t], label, int(t), state))

    if change_pts:
        xs, ys, zs, labels, idxs, states = zip(*change_pts)
        fig.add_trace(go.Scatter3d(
            x=xs, y=ys, z=zs,
            mode='markers',
            marker=dict(size=7, color='red', symbol='diamond',
                        line=dict(color='black', width=1)),
            name='Gripper open/close',
            customdata=list(zip(labels, idxs, states)),
            hovertemplate=('<b>Gripper %{customdata[2]}</b><br>'
                           '%{customdata[0]} @ point %{customdata[1]}<extra></extra>')
        ))

    # Always last, always empty to start: a viewer draws a gripper_glyph here,
    # at the waypoint whose camera frame it is showing (see CURRENT_FRAME_TRACE).
    fig.add_trace(go.Scatter3d(
        x=[], y=[], z=[], mode='lines',
        line=dict(color='#111', width=6),
        name=CURRENT_FRAME_TRACE, showlegend=False, hoverinfo='skip'))

    fig.update_layout(
        title='End-Effector Trajectory',
        scene=dict(aspectmode='data', xaxis_title='X', yaxis_title='Y', zaxis_title='Z',
                   # Plotly's default eye (1.25) crops a data-aspect scene; back
                   # off so the whole trajectory is in view before any dragging.
                   camera=dict(eye=dict(x=2.5, y=2.5, z=1.7))),
        legend=dict(orientation="h", yanchor="bottom", y=1.02),
        margin=dict(l=0, r=0, b=0, t=60),
    )
    return fig


def minimal_trajectory_fig(traj, max_points: int = 80):
    """Quick-to-load minimalist variant of the 3D trajectory plot:
    downsampled to <= max_points, progress-colored line, no colorbar, no
    axes, no hover -- meant as a small embedded preview."""
    step = max(1, traj.shape[1] // max_points)
    t = traj[:, ::step]
    idx = np.arange(t.shape[1])
    fig = go.Figure(go.Scatter3d(
        x=t[0], y=t[1], z=t[2],
        mode='lines+markers',
        marker=dict(size=3, color=idx, colorscale='Viridis', showscale=False),
        line=dict(color='royalblue', width=3),
        hoverinfo='skip',
    ))
    fig.update_layout(
        scene=dict(aspectmode='data',
                   xaxis=dict(visible=False), yaxis=dict(visible=False), zaxis=dict(visible=False)),
        margin=dict(l=0, r=0, b=0, t=0),
        showlegend=False,
    )
    return fig


def minimal_trajectory_png(skill_files, size_px: int = 240, max_points: int = 80, cache_dir=None):
    """Render the minimalist 3D trajectory to a small PNG and return its path.

    skill_files is one filename or a list: a double-object skill's part1 and
    part2 recordings are overlaid, each with its own line color.
    Rendered with matplotlib (Agg, no pyplot) instead of kaleido, so it needs
    no browser and takes ~0.1 s; only the 'traj' member of each npz is
    decompressed. The PNG is cached in TRAJECTORIES_DIR/.thumbnails and
    reused while it is newer than the npz(s), so repeated loads are instant."""
    if isinstance(skill_files, str):
        skill_files = [skill_files]
    paths = [f if (os.path.isabs(f) or os.path.sep in f) else os.path.join(TRAJECTORIES_DIR, f)
             for f in skill_files]
    cache_dir = cache_dir or os.path.join(TRAJECTORIES_DIR, ".thumbnails")
    os.makedirs(cache_dir, exist_ok=True)
    name = "__".join(os.path.basename(p).replace(".npz", "") for p in paths)
    out = os.path.join(cache_dir, f"{name}.png")
    if os.path.isfile(out) and os.path.getmtime(out) >= max(os.path.getmtime(p) for p in paths):
        return out

    trajs = []
    for p in paths:
        npz = np.load(p)
        traj = npz['traj'] if 'traj' in npz.files else npz['traj.npy']
        if traj.ndim == 2 and traj.shape[1] >= 2:
            step = max(1, traj.shape[1] // max_points)
            trajs.append(traj[:, ::step])
    if not trajs:
        return None

    from matplotlib.figure import Figure  # Agg canvas, thread-safe (no pyplot)
    fig = Figure(figsize=(size_px / 100, size_px / 100), dpi=100)
    ax = fig.add_subplot(projection='3d')
    for i, t in enumerate(trajs):
        color = TRAJ_COLORS[i % len(TRAJ_COLORS)]
        ax.plot(t[0], t[1], t[2], color=color, linewidth=1.5)
        ax.scatter(t[0], t[1], t[2], color=color, s=6,
                   alpha=np.linspace(0.35, 1.0, t.shape[1]))  # progress as opacity
    ax.set_axis_off()
    allt = np.concatenate(trajs, axis=1)
    try:
        ax.set_box_aspect(tuple(max(float(np.ptp(allt[i])), 1e-6) for i in range(3)))  # aspectmode='data'
    except Exception:
        pass
    fig.savefig(out, transparent=True, bbox_inches='tight', pad_inches=0)
    return out


def _find_free_port(preferred=0):
    """Return an available TCP port (0 => ask OS to choose)."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", preferred))
        return s.getsockname()[1]

def _fig_from_traj(traj):
    """Build the SAME 3D trajectory figure you already use."""
    point_indices = np.arange(traj.shape[1])
    fig = go.Figure(
        data=[go.Scatter3d(
            x=traj[0,:], y=traj[1,:], z=traj[2,:],
            mode='markers+lines',
            marker=dict(size=4, color=point_indices, colorscale='Viridis',
                        colorbar=dict(title='Point Index')),
            line=dict(color='royalblue', width=2),
            customdata=point_indices,
            hovertemplate=('<b>Point %{customdata}</b><br>'
                           'X: %{x:.2f}<br>Y: %{y:.2f}<br>Z: %{z:.2f}<extra></extra>')
        )]
    )
    fig.update_layout(
        title='End-Effector Trajectory',
        scene=dict(aspectmode='data', xaxis_title='X', yaxis_title='Y', zaxis_title='Z'),
        margin=TRAJECTORYPLOT_MARGINS,
    )

    return fig

def save_trajectory_png(skill_file, out_png="trajectory.png"):
    if os.path.isabs(skill_file) or os.path.sep in skill_file:
        npz = np.load(skill_file); traj = npz['traj.npy']
    else:
        traj = load_skill_data(skill_file)['traj']
    fig = _fig_from_traj(traj)
    fig.write_image(out_png, scale=2)  # needs kaleido
    return out_png

def show_skill(skill_file, port=8090, inline=True, height=520, debug=False):
    """
    Minimal viewer for a single skill (no gripper, no template).
    Uses existing load_skill_data(...) and numpy_to_base64(...).

    Usage:
        show_skill("my_skill.npz")                # file inside TRAJECTORIES_DIR
        show_skill("/abs/path/to/my_skill.npz")   # absolute path
    """
    if skill_file[-4:] != ".npz":
        skill_file = skill_file + ".npz"
    
    offset = 0
    if "_branch_at_" in skill_file:
        skill_file_without_npz = skill_file[:-4]
        after_branch_at = skill_file_without_npz.split("_branch_at_")[-1]
        offset_str = after_branch_at.split("_")[0]
        offset = int(offset_str)

    # --- Load data (supports filename OR absolute path) ---
    if os.path.isabs(skill_file) or os.path.sep in skill_file:
        data_npz = np.load(skill_file)
        if "tag" in data.keys():
            tag = str(data['tag.npy'])
        else:
            tag = "no tag"
        data = {
            'traj': data_npz['traj.npy'],
            'images': data_npz['img.npy'],
            'tag': tag,
        }
        skill_label = os.path.basename(skill_file)
    else:
        data = load_skill_data(skill_file)  # existing helper (expects filename in TRAJECTORIES_DIR)
        skill_label = skill_file

    traj = data['traj']
    if traj.ndim == 1: return
    images = data['images']
    grip = data['grip'].squeeze()
    tag = ""
    if "tag" in data.keys():
        tag = str(data['tag'])
    
    # Unique identity for this app instance
    app_id = str(uuid.uuid4())[:8]
    base_path = f"/{app_id}/"
    if port is None:
        port = _find_free_port(0)

    point_indices = np.arange(traj.shape[1]) + offset
    trajectory_fig = go.Figure(
        data=[go.Scatter3d(
            x=traj[0,:],
            y=traj[1,:],
            z=traj[2,:],
            mode='markers+lines',
            marker=dict(
                size=4,
                color=point_indices,
                colorscale='Viridis',
                colorbar=dict(title='Point Index')
            ),
            line=dict(color='royalblue', width=2),
            customdata=point_indices,
            hovertemplate='<b>Point %{customdata}</b><br>' +
                          'X: %{x:.2f}<br>Y: %{y:.2f}<br>Z: %{z:.2f}<extra></extra>'
        )]
    )
    trajectory_fig.update_layout(
        title=f'{skill_label} End-Effector Trajectory',
        scene=dict(
            aspectmode='data',
            xaxis_title='X',
            yaxis_title='Y',
            zaxis_title='Z'
        ),
        margin=dict(l=0, r=0, b=0, t=40),
    )

    grip = np.asarray(grip, dtype=float)
    x = np.asarray(point_indices)

    # Define state and labels
    OPEN_THRESHOLD = 0.04  # anything >= this counts as open
    is_open = grip >= OPEN_THRESHOLD
    state_txt = np.where(is_open, "Open", "Closed")

    transition_idx = np.where(np.diff(is_open.astype(int)) != 0)[0] + 1

    # Compute "time since last change" (in points) for hover
    last_change = np.zeros_like(x, dtype=int)
    if len(x) > 0:
        lc = 0
        for i in range(len(x)):
            if i in transition_idx:
                lc = 0
            last_change[i] = lc
            lc += 1

    band_colors = np.where(is_open, "rgba(0,150,0,0.15)", "rgba(120,120,120,0.15)")
    band_trace = go.Bar(
        x=x,
        y=np.full_like(grip, 0.09, dtype=float),   # a constant tall bar spanning the y-range
        base=0.0,
        marker=dict(color=band_colors, line=dict(width=0)),
        hoverinfo="skip",
        opacity=1.0,
        showlegend=False
    )

    # Step line for the actual measurements
    main_trace = go.Scatter(
        x=x,
        y=grip,
        mode="lines",
        line=dict(width=2),
        line_shape="hv",  # step-like: hold value, then vertical jump
        name="Gripper",
        customdata=np.stack([state_txt, last_change], axis=1),
        hovertemplate=(
            "Point: %{x}<br>"
            "Value: %{y:.2f} m<br>"
            "State: %{customdata[0]}<br>"
            "Since last change: %{customdata[1]} step(s)"
            "<extra></extra>"
        )
    )

    # Markers only at transitions
    transition_trace = go.Scatter(
        x=x[transition_idx] if transition_idx.size else [],
        y=grip[transition_idx] if transition_idx.size else [],
        mode="markers",
        marker=dict(size=8, symbol="circle"),
        name="State change",
        hovertemplate="Point: %{x}<br>Value: %{y:.2f} m<br><b>State changed here</b><extra></extra>"
    )

    # (Optional) tiny rug to emphasize state on the baseline
    rug_trace = go.Scatter(
        x=x,
        y=np.full_like(grip, -0.002, dtype=float),  # just below zero line
        mode="markers",
        marker=dict(size=4, symbol="line-ns"),
        name="State rug",
        hoverinfo="skip",
        showlegend=False,
        opacity=0.6
    )

    # --- Figure ---
    gripper_fig = go.Figure(data=[band_trace, main_trace, transition_trace, rug_trace])

    gripper_fig.update_layout(
        title="",
        margin=dict(l=0, r=0, b=0, t=40),
        hovermode="x unified",
        xaxis=dict(
            title="Point Index",
            rangeslider=dict(visible=True),  # quick zooming
            showspikes=True,
            spikemode="across",
            spikesnap="cursor",
            showgrid=False
        ),
        yaxis=dict(
            title="Gripper [m]",
            range=[-0.01, 0.09],
            tickmode="array",
            tickvals=[0.0, 0.08],
            ticktext=["Closed", "Open"],
            zeroline=True,
            zerolinewidth=1,
            showgrid=True,
            gridwidth=1
        ),
        legend=dict(orientation="h", x=0, y=1.1)
    )

    # Subtle horizontal reference lines (0 and 0.08)
    gripper_fig.add_hline(y=0.0, line_width=1, line_dash="dot")
    gripper_fig.add_hline(y=0.08, line_width=1, line_dash="dot")


    # --- Minimal Dash app layout: left = trajectory, right = image ---
    app = dash.Dash(__name__ + "_show_skill")
    app.layout = html.Div([
        html.Div([
            dcc.Graph(
                id='traj-3d',
                figure=trajectory_fig,
                style={'width': '49%', 'display': 'inline-block', 'verticalAlign': 'top'}
            ),
            html.Div([
                html.Div([
                    html.Span(
                        tag if tag else "No tag",
                        id="tag-pill",
                        style={
                            "display": "inline-block",
                            "padding": "4px 10px",
                            "borderRadius": "999px",
                            "background": "#f0f4ff" if tag else "#f6f6f6",
                            "color": "#1f3a93" if tag else "#999",
                            "fontFamily": "ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, 'Liberation Mono'",
                            "fontSize": "12px",
                            "border": "1px solid #dbe3ff" if tag else "1px solid #e9e9e9",
                            "marginRight": "8px"
                        }
                    ),
                    html.Button("▶ Play 60 FPS", id="play-btn",
                                style={"padding": "6px 10px", "borderRadius": "8px", "border": "1px solid #ddd",
                                    "background": "#fafafa", "cursor": "pointer", "fontWeight": 600}),
                    html.Span(id="play-status", style={"marginLeft": "10px", "color": "#666", "fontSize": "12px"})
                ], style={"marginBottom": "8px"}),
                html.Img(
                    id='trajectory-image',
                    style={
                        'width': '100%',
                        'maxHeight': '500px',
                        'objectFit': 'contain',
                        'border': '1px solid #ddd',
                        'borderRadius': '8px'
                    }
                ),
                html.Div(id='img-caption', style={'marginTop':'6px','fontFamily':'monospace','fontSize':'12px','color':'#666'}),
                dcc.Graph(
                    id="gripper-plot",
                    figure=gripper_fig,
                    style={"height": "240px", "width": "100%", "display": "inline-block"}
                )
            ], style={'width': '49%', 'display': 'inline-block', 'paddingLeft': '12px', 'verticalAlign': 'top'})
        ])
    ], style={'padding': '6px', 'background-color': '#FFF'})

    @app.callback(
        Output("play-status", "children"),
        Input("play-btn", "n_clicks"),
        prevent_initial_call=True
    )
    def _play_clicked(n_clicks):
        import uuid, time
        import numpy as np
        try:
            vid = images
            # Accept (N, H, W) or (N, C, H, W) -> take first channel
            if vid.ndim == 4:
                vid = vid[:, 0]

            # Ensure memory layout is friendly for cv2 (avoids occasional no-op on subsequent runs)
            vid = np.ascontiguousarray(vid)

            # --- Hard reset OpenCV windows from prior run ---
            try:
                import cv2
                cv2.destroyAllWindows()
                # Pump the event loop a few times so HighGUI really releases
                for _ in range(3):
                    cv2.waitKey(1)
                    time.sleep(0.01)
            except Exception:
                pass

            # Unique window name so OpenCV reliably re-opens each time
            win = f"Video-{uuid.uuid4().hex[:6]}"

            # Play synchronously; when it returns, playback has finished
            play_video(vid, fps=60, window_name=win, backend="cv2", scale=3.0)

            # Extra cleanup to allow immediate re-run
            try:
                import cv2
                cv2.destroyWindow(win)
                for _ in range(3):
                    cv2.waitKey(1)
                    time.sleep(0.01)
            except Exception:
                pass

            return "Finished playing."
        except Exception as e:
            print("play_video error:", e)
            return f"Playback error: {e}"

    @app.callback(
        Output('trajectory-image', 'src'),
        Output('img-caption', 'children'),
        Input('traj-3d', 'clickData'),
        prevent_initial_call=False  # starts empty until you click
    )
    def update_clicked_image(clickData):
        if not clickData or not clickData.get('points'):
            return None, "Click a waypoint to view its image."
        try:
            # Use the same customdata index we attached to the points
            point_idx = clickData['points'][0].get('customdata', clickData['points'][0].get('pointNumber', 0))
            point_idx = int(point_idx)

            img_array = images[point_idx]
            # Handle grayscale image (H,W) or (N,H,W) by taking first frame
            if len(img_array.shape) == 3:
                img_array = img_array[0]

            return numpy_to_base64(img_array), f"Point index: {point_idx}"
        except Exception as e:
            print(f"Error updating image: {e}")
            return None, "Failed to render image for this waypoint."

    # app.run(debug=debug, host='0.0.0.0', port=port)
    run_kwargs = dict(
        debug=debug,
        host="127.0.0.1", port=port,
        jupyter_width="800px",      # optional
        dev_tools_ui=False,         # <-- hide the bottom Dev Tools panel
        dev_tools_hot_reload=False  # avoid hijacking previous iframes
    )

    # try:
    # Dash >= 2.11 supports jupyter_* args
    if inline:
        app.run(jupyter_mode="inline", jupyter_height=height, **run_kwargs)
    else:
        app.run(jupyter_mode="tab", **run_kwargs)
    
    # except TypeError:
    #     # Fallback to JupyterDash if older stack:
    #     from jupyter_dash import JupyterDash
    #     app2 = JupyterDash(name=f"show_skill_{app_id}",
    #                        routes_pathname_prefix=base_path,
    #                        requests_pathname_prefix=base_path)
    #     app2.layout = app.layout
    #     app2.callback_map = app.callback_map
    #     app2.run_server(mode="inline" if inline else "tab",
    #                     height=height if inline else None,
    #                     **run_kwargs)

    # Return the URL in case you want to open it manually too
    return f"http://127.0.0.1:{port}{base_path}"

def run():
    app.run(debug=True, host='0.0.0.0', port=8076)
    

if __name__ == '__main__':
    run()


def play_video(frames, fps=60, window_name="Video", scale=1.0, backend="auto"):
    """
    Play a grayscale video from a numpy array of shape (N, H, W) dtype=uint8.
    Closes the window when finished (or if user presses Esc / q) and returns.

    Args:
        frames: np.ndarray, shape (N, H, W), dtype uint8
        fps: target frames per second (default 60)
        window_name: window title for display
        scale: optional display scale factor (e.g., 2.0 to double size)
        backend: "auto" | "cv2" | "matplotlib"
    """
    import numpy as np
    import time

    # Basic validation/fixes
    if not isinstance(frames, np.ndarray) or frames.ndim != 3:
        raise ValueError("frames must be a (N, H, W) numpy array")
    if frames.shape[0] == 0:
        print("no frames")
        return
    if frames.dtype != np.uint8:
        # convert safely (keeps 0-255 range)
        frames = np.clip(frames, 0, 255).astype(np.uint8)

    N, H, W = frames.shape
    interval = 1.0 / float(fps)

    def _cv2_player():
        import cv2
        # Optionally scale for visibility
        if scale != 1.0:
            new_size = (int(W * scale), int(H * scale))

        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, int(W * scale), int(H * scale))

        start = time.perf_counter()
        for i in range(N):
            img = frames[i]
            if scale != 1.0:
                img = cv2.resize(img, new_size, interpolation=cv2.INTER_NEAREST)

            # Grayscale display
            cv2.imshow(window_name, img)

            # Drift-corrected pacing
            target = start + (i + 1) * interval
            # process events; keep waitKey tiny to avoid blocking
            k = cv2.waitKey(1) & 0xFF
            if k in (27, ord('q')):  # ESC or q quits early
                break

            sleep_time = target - time.perf_counter()
            if sleep_time > 0:
                # time.sleep is coarse but good enough for 60 FPS
                time.sleep(sleep_time)

        cv2.destroyWindow(window_name)

    def _mpl_player():
        # Matplotlib is usually slower; use only if cv2 isn't available
        import matplotlib.pyplot as plt

        plt.ion()
        fig = plt.figure(window_name)
        ax = fig.add_subplot(111)
        im = ax.imshow(frames[0], cmap="gray", vmin=0, vmax=255, interpolation="nearest")
        ax.set_axis_off()
        fig.canvas.draw()
        fig.canvas.flush_events()

        start = time.perf_counter()
        for i in range(N):
            im.set_data(frames[i])
            fig.canvas.draw()
            fig.canvas.flush_events()
            target = start + (i + 1) * interval
            sleep_time = target - time.perf_counter()
            if sleep_time > 0:
                plt.pause(sleep_time)  # non-blocking pause

        plt.ioff()
        plt.close(fig)

    if backend in ("auto", "cv2"):
        try:
            import cv2  # noqa: F401
            _cv2_player()
            return
        except Exception:
            if backend == "cv2":
                raise  # user explicitly requested cv2; bubble the error

    # Fallback to matplotlib
    _mpl_player()