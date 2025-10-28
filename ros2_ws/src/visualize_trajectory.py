#!/usr/bin/env python3
"""
Trajectory Visualization with Plotly

Creates interactive visualization showing:
- Occupancy grid map
- Odometry trajectory (drifting)
- Map trajectory (corrected by localization)
- Waypoints
- Landmarks
- Drift analysis
"""

import json
import numpy as np
from PIL import Image
import plotly.graph_objects as go
from plotly.subplots import make_subplots


def load_map(map_path='/shared/maps/landmark_map.pgm'):
    """Load occupancy grid map"""
    img = Image.open(map_path)
    map_data = np.array(img)
    # Flip vertically to match ROS coordinates
    map_data = np.flipud(map_data)
    return map_data


def create_visualization(data_path='/shared/student_work/trajectory_data.json'):
    """Create comprehensive Plotly visualization"""

    # Load data
    with open(data_path, 'r') as f:
        data = json.load(f)

    odom_traj = np.array(data['odom_trajectory'])
    map_traj = np.array(data['map_trajectory'])
    waypoints = data['waypoints']
    landmarks = data['landmarks']

    # Load map
    map_img = load_map()

    # Map parameters (from landmark_map.yaml)
    resolution = 0.05  # m/pixel
    origin_x, origin_y = -5.0, -5.0  # meters

    # Create figure
    fig = go.Figure()

    # Add occupancy grid map as background
    fig.add_trace(go.Heatmap(
        z=map_img,
        x=np.linspace(origin_x, origin_x + map_img.shape[1] * resolution, map_img.shape[1]),
        y=np.linspace(origin_y, origin_y + map_img.shape[0] * resolution, map_img.shape[0]),
        colorscale='Greys',
        showscale=False,
        name='Map',
        hoverinfo='skip'
    ))

    # Add landmarks (cylinders)
    for landmark in landmarks:
        theta = np.linspace(0, 2*np.pi, 50)
        x_circle = landmark['x'] + landmark['radius'] * np.cos(theta)
        y_circle = landmark['y'] + landmark['radius'] * np.sin(theta)

        fig.add_trace(go.Scatter(
            x=x_circle,
            y=y_circle,
            mode='lines',
            fill='toself',
            fillcolor=landmark['color'],
            opacity=0.5,
            line=dict(color=landmark['color'], width=2),
            name=f"Landmark: {landmark['name']}",
            hovertext=f"{landmark['name']}<br>({landmark['x']:.1f}, {landmark['y']:.1f})"
        ))

    # Add waypoints
    wp_x = [wp['x'] for wp in waypoints]
    wp_y = [wp['y'] for wp in waypoints]
    wp_names = [wp['name'] for wp in waypoints]

    fig.add_trace(go.Scatter(
        x=wp_x,
        y=wp_y,
        mode='markers+text',
        marker=dict(size=12, color='purple', symbol='star', line=dict(width=2, color='white')),
        text=[f"WP{i+1}" for i in range(len(waypoints))],
        textposition='top center',
        name='Waypoints',
        hovertext=wp_names
    ))

    # Add odometry trajectory (drifting)
    if len(odom_traj) > 0:
        fig.add_trace(go.Scatter(
            x=odom_traj[:, 0],
            y=odom_traj[:, 1],
            mode='lines',
            line=dict(color='red', width=3),
            name='Odometry Path (Drifting)',
            hovertext=[f"t={t:.1f}s" for t in odom_traj[:, 2]]
        ))

        # Add start/end markers for odom
        fig.add_trace(go.Scatter(
            x=[odom_traj[0, 0], odom_traj[-1, 0]],
            y=[odom_traj[0, 1], odom_traj[-1, 1]],
            mode='markers',
            marker=dict(size=15, color=['green', 'red'], symbol=['circle', 'x'],
                       line=dict(width=2, color='white')),
            name='Odom Start/End',
            hovertext=['Odom Start', 'Odom End']
        ))

    # Add map trajectory (corrected)
    if len(map_traj) > 0:
        fig.add_trace(go.Scatter(
            x=map_traj[:, 0],
            y=map_traj[:, 1],
            mode='lines',
            line=dict(color='lime', width=3, dash='dash'),
            name='Map Path (Corrected)',
            hovertext=[f"t={t:.1f}s" for t in map_traj[:, 2]]
        ))

        # Add start/end markers for map
        fig.add_trace(go.Scatter(
            x=[map_traj[0, 0], map_traj[-1, 0]],
            y=[map_traj[0, 1], map_traj[-1, 1]],
            mode='markers',
            marker=dict(size=15, color=['green', 'lime'], symbol=['circle', 'x'],
                       line=dict(width=2, color='black')),
            name='Map Start/End',
            hovertext=['Map Start', 'Map End']
        ))

    # Calculate drift metrics
    if len(odom_traj) > 0 and len(map_traj) > 0:
        odom_start = odom_traj[0, :2]
        odom_end = odom_traj[-1, :2]
        map_start = map_traj[0, :2]
        map_end = map_traj[-1, :2]

        odom_drift = np.linalg.norm(odom_end - odom_start)
        map_error = np.linalg.norm(map_end - map_start)
        improvement = (1 - map_error / odom_drift) * 100 if odom_drift > 0 else 0

        # Add drift vector visualization
        fig.add_trace(go.Scatter(
            x=[odom_end[0], map_end[0]],
            y=[odom_end[1], map_end[1]],
            mode='lines+markers',
            line=dict(color='orange', width=2, dash='dot'),
            marker=dict(size=10, color='orange'),
            name='Drift Correction',
            hovertext=f'Drift: {odom_drift:.3f}m → {map_error:.3f}m'
        ))

    # Layout configuration
    fig.update_layout(
        title={
            'text': 'Exercise 3b: Nav2 Waypoint Navigation with Drift Analysis<br>' +
                    f'<sub>Odom Drift: {odom_drift:.3f}m | Map Error: {map_error:.3f}m | ' +
                    f'Improvement: {improvement:.1f}%</sub>',
            'x': 0.5,
            'xanchor': 'center',
            'font': {'size': 20}
        },
        xaxis=dict(title='X Position (m)', scaleanchor='y', scaleratio=1),
        yaxis=dict(title='Y Position (m)'),
        width=1200,
        height=1000,
        hovermode='closest',
        showlegend=True,
        legend=dict(x=1.02, y=1, xanchor='left', yanchor='top'),
        plot_bgcolor='white',
        paper_bgcolor='white'
    )

    # Save HTML
    output_path = '/shared/student_work/trajectory_visualization.html'
    fig.write_html(output_path)
    print(f'✅ Visualization saved to: {output_path}')
    print(f'📊 Odometry drift: {odom_drift:.3f} m')
    print(f'📊 Map frame error: {map_error:.3f} m')
    print(f'📊 Localization improvement: {improvement:.1f}%')

    return fig


if __name__ == '__main__':
    create_visualization()
