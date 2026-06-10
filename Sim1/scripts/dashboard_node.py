#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from dash import Dash, html, dcc
from dash.dependencies import Input, Output
import plotly.graph_objs as go
from threading import Thread
import pandas as pd
from collections import deque

# إعداد البيانات (بنشيل آخر 50 قراءة)
times = deque(maxlen=50)
joint1_data = deque(maxlen=50)
joint2_data = deque(maxlen=50)
joint3_data = deque(maxlen=50)

class DashNode(Node):
    def __init__(self):
        super().__init__('dash_node')
        self.create_subscription(JointState, '/slave/joint_states', self.callback, 10)

    def callback(self, msg):
        times.append(msg.header.stamp.sec)
        joint1_data.append(msg.position[0])
        joint2_data.append(msg.position[1])
        joint3_data.append(msg.position[2])

# إعداد Dash App
app = Dash(__name__)
app.layout = html.Div(style={'backgroundColor': '#1e1e2f', 'color': '#ffffff', 'padding': '20px'}, children=[
    html.H1("Surgical Robot Digital Twin Dashboard", style={'textAlign': 'center', 'color': '#bb86fc'}),
    
    dcc.Graph(id='live-graph'),
    dcc.Interval(id='graph-update', interval=500, n_intervals=0), # تحديث كل نص ثانية
])

@app.callback(Output('live-graph', 'figure'), [Input('graph-update', 'n_intervals')])
def update_graph(n):
    fig = go.Figure()
    fig.add_trace(go.Scatter(x=list(range(len(joint1_data))), y=list(joint1_data), name='Base (J1)', line=dict(color='#03dac6')))
    fig.add_trace(go.Scatter(x=list(range(len(joint2_data))), y=list(joint2_data), name='Shoulder (J2)', line=dict(color='#bb86fc')))
    fig.add_trace(go.Scatter(x=list(range(len(joint3_data))), y=list(joint3_data), name='Elbow (J3)', line=dict(color='#cf6679')))
    
    fig.update_layout(title='Real-time Joint Angles (Radians)',
                      plot_bgcolor='#1e1e2f', paper_bgcolor='#1e1e2f',
                      font=dict(color='#ffffff'), xaxis=dict(showgrid=False), yaxis=dict(showgrid=True, gridcolor='#333333'))
    return fig

def run_ros():
    rclpy.init()
    node = DashNode()
    rclpy.spin(node)

if __name__ == '__main__':
    # تشغيل ROS في Thread منفصل عشان الـ Dashboard متقفش
    Thread(target=run_ros).start()
    app.run(debug=False, port=8050)
