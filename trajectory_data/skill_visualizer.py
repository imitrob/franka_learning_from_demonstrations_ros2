import dash
from dash import dcc, html, Input, Output, State
import plotly.graph_objs as go
import numpy as np
import os
from PIL import Image
import base64
import io

# Configuration
import trajectory_data, object_localization
TRAJECTORIES_DIR = f"/home/imitlearn/petr_sandbox/saw_ws/src/trajectory_data/trajectories/quantitative_study"
TRAJECTORIES_DIR = f"{trajectory_data.package_path}/trajectories"
CFG_DIR = f"/home/imitlearn/petr_sandbox/saw_ws/src/ILeSiA/franka_risk_aware_learning_from_demonstrations/object_localization/cfg"
CFG_DIR = f"{object_localization.package_path}/cfg"

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
        )
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
    return {
        'traj': data['traj.npy'],
        'grip': data['grip.npy'],
        'images': data['img.npy']
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
        margin=dict(l=0, r=0, b=0, t=40)
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

if __name__ == '__main__':
    app.run(debug=True, port=8050)