import os
import json
import networkx as nx
import plotly.graph_objects as go
from dash import Dash, html, dcc, Input, Output, State
import dash_bootstrap_components as dbc
from dash.dependencies import ClientsideFunction

# File path setup
script_dir = os.path.dirname(os.path.abspath(__file__))
logs_dir = os.path.abspath(os.path.join(script_dir, "..", "logs"))

date_time = "2025-05-13_14-48-37"
model_name = "rvnn_iter250"
count = 1

json_path = os.path.join(logs_dir, date_time, model_name, f"mcts_tree_{count}.json")
output_path = os.path.join(logs_dir, date_time, model_name, f"mcts_tree_{count}.html")

# Load JSON data
with open(json_path, "r") as f:
    data = json.load(f)

# Build graph
G = nx.DiGraph()
node_pos = {}
for i, node in enumerate(data["nodes"]):
    G.add_node(node["id"], **node)
for edge in data["edges"]:
    G.add_edge(edge["source"], edge["target"], **edge)

# Use Graphviz layout if available (top-down)
try:
    pos = nx.nx_agraph.graphviz_layout(G, prog='dot')
except:
    pos = nx.spring_layout(G, seed=42)

node_x, node_y, node_text, node_ids = [], [], [], []
for node_id, (x, y) in pos.items():
    node = G.nodes[node_id]
    node_x.append(x)
    node_y.append(y)  # NO flip → top-down preserved
    node_text.append(f"ID: {node_id}<br>State: {node['state']}<br>Reward: {node['reward']}<br>Terminal: {node['is_terminal']}")
    node_ids.append(node_id)

edge_x, edge_y, edge_hover_x, edge_hover_y, edge_text, edge_ids = [], [], [], [], [], []
for (u, v, attr) in G.edges(data=True):
    x0, y0 = pos[u]
    x1, y1 = pos[v]
    edge_x += [x0, x1, None]
    edge_y += [y0, y1, None]
    edge_hover_x.append((x0 + x1) / 2)
    edge_hover_y.append((y0 + y1) / 2)
    edge_ids.append(f"{u}-{v}")
    edge_text.append(f"Edge {u}→{v}<br>Action: {attr['action']}<br>Visits: {attr['visits']}<br>Q: {attr['q']:.3f}<br>Prior: {attr['prior']:.6f}")


# Create Plotly Figure
fig = go.Figure()

# Edges (lines only, no hover)
fig.add_trace(go.Scatter(
    x=edge_x, y=edge_y,
    mode='lines',
    line=dict(width=1, color='#888'),
    hoverinfo='none',
    name='edges'
))

# Edge hover points (invisible points with hover)
fig.add_trace(go.Scatter(
    x=edge_hover_x, y=edge_hover_y,
    mode='markers',
    marker=dict(size=10, color='rgba(0,0,0,0)'),  # transparent
    hoverinfo='text',
    hovertext=edge_text,
    customdata=edge_ids,
    name='edge-hover'
))


# Nodes
# Determine colors based on terminal status
node_colors = [
    'tomato' if G.nodes[n]['is_terminal'] else 'lightblue'
    for n in node_ids
]

# Nodes with color-coded terminal status
fig.add_trace(go.Scatter(
    x=node_x, y=node_y,
    mode='markers+text',
    marker=dict(
        size=20,
        color=node_colors,
        line=dict(width=2, color='darkblue')
    ),
    text=[str(i) for i in node_ids],
    textposition="top center",
    hovertext=node_text,
    hoverinfo='text',
    customdata=node_ids,
    name='nodes'
))

fig.update_layout(
    showlegend=False,
    margin=dict(l=10, r=10, t=10, b=10),
    hovermode='closest',
    plot_bgcolor='white',
    paper_bgcolor='white',
    xaxis=dict(visible=False),
    yaxis=dict(visible=False)
)

# Dash App
app = Dash(__name__, external_stylesheets=[dbc.themes.BOOTSTRAP])
app.layout = dbc.Container([
    html.H3("Monte Carlo Search Tree Visualization"),
    dcc.Graph(
        id='mcts-graph',
        figure=fig,
        style={'height': '800px'}
    ),
    dcc.Store(id='pinned-annotations', data=[])
])

# Store clicked nodes/edges
clicked_items = set()

@app.callback(
    Output('mcts-graph', 'figure'),
    Output('pinned-annotations', 'data'),
    Input('mcts-graph', 'clickData'),
    State('mcts-graph', 'figure'),
    State('pinned-annotations', 'data'),
)
def update_annotations(clickData, fig_dict, pinned_annotations):
    if not clickData:
        return fig_dict, pinned_annotations

    fig = go.Figure(fig_dict)
    point = clickData['points'][0]
    x, y = point['x'], point['y']
    label = point.get('text')
    customdata = point.get('customdata')

    # Define annotation text based on whether it's node or edge
    if isinstance(customdata, int):
        node = G.nodes[customdata]
        annotation_text = f"Node {customdata}State: {node['state']}<br>Reward: {node['reward']}<br>Terminal: {node['is_terminal']}"
        key = f"Node-{customdata}"
    else:
        u, v = map(int, customdata.split('-'))
        edge = G[u][v]
        annotation_text = f"Edge {u}->{v}<br>Action: {edge['action']}<br>Visits: {edge['visits']}<br>Q: {edge['q']:.3f}<br>Prior: {edge['prior']:.6f}"
        key = f"Edge-{u}-{v}"

    # Check if annotation is already pinned
    if key in [ann['key'] for ann in pinned_annotations]:
        # Remove it
        pinned_annotations = [ann for ann in pinned_annotations if ann['key'] != key]
    else:
        # Add it
        pinned_annotations.append({
            'key': key,
            'x': x,
            'y': y,
            'text': annotation_text
        })

    # Rebuild all annotations
    fig.update_layout(annotations=[
        dict(
            x=ann['x'],
            y=ann['y'],
            xref="x",
            yref="y",
            text=ann['text'],
            showarrow=True,
            arrowhead=4,
            ax=20,
            ay=-20,
            bgcolor="white",
            bordercolor="black",
            borderwidth=1,
            opacity=0.9
        ) for ann in pinned_annotations
    ])

    return fig, pinned_annotations

def display_click_info(clickData, current_output):
    global clicked_items
    if not clickData:
        return current_output

    point = clickData['points'][0]
    point_id = point['customdata']
    label = point.get('text')

    if isinstance(point_id, int):  # It's a node
        node = G.nodes[point_id]
        info = f"[Node {point_id}]\nState: {node['state']}\nTerminal: {node['is_terminal']}"
        key = f"Node-{point_id}"
    else:
        u, v = map(int, point_id.split('-'))
        edge = G[u][v]
        info = f"[Edge {u}->{v}]\nAction: {edge['action']}\nVisits: {edge['visits']}\nQ: {edge['q']:.3f}\nPrior: {edge['prior']:.6f}"
        key = f"Edge-{u}-{v}"

    if key in clicked_items:
        clicked_items.remove(key)
    else:
        clicked_items.add(key)

    output_text = "\n\n".join(
        f"[{k}]\n{display_click_info_by_key(k)}"
        for k in sorted(clicked_items)
    )
    return output_text


def display_click_info_by_key(key):
    if key.startswith("Node-"):
        node_id = int(key.split("-")[1])
        node = G.nodes[node_id]
        return f"State: {node['state']}\nTerminal: {node['is_terminal']}"
    elif key.startswith("Edge-"):
        parts = key.split("-")
        u, v = int(parts[1]), int(parts[2])
        edge = G[u][v]
        return f"Action: {edge['action']}\nVisits: {edge['visits']}\nQ: {edge['q']:.3f}\nPrior: {edge['prior']:.6f}"
    return ""

if __name__ == '__main__':
    app.run(debug=True)
