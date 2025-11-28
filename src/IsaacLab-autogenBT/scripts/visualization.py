import os
import json
import networkx as nx
import plotly.graph_objects as go
from dash import Dash, html, dcc, Input, Output, State
import dash_bootstrap_components as dbc

# ---------- config (edit if needed) ----------
script_dir = os.path.dirname(os.path.abspath(__file__))
logs_dir = os.path.abspath(os.path.join(script_dir, "..", "logs"))

date_time = "2025-11-27_14-30-04"
model_name = "rvnn_iter000"
count = 1

json_path = os.path.join(logs_dir, date_time, model_name, f"mcts_tree_{count}.json")
# ------------------------------------------------

# Load JSON data
if not os.path.exists(json_path):
    raise FileNotFoundError(f"Could not find file: {json_path}")

with open(json_path, "r") as f:
    data = json.load(f)

# Build graph
G = nx.DiGraph()
for node in data.get("nodes", []):
    # Ensure id is int if numeric-looking
    node_id = int(node["id"]) if isinstance(node["id"], (int, str)) and str(node["id"]).isdigit() else node["id"]
    G.add_node(node_id, **node)

for edge in data.get("edges", []):
    src = int(edge["source"]) if str(edge["source"]).isdigit() else edge["source"]
    tgt = int(edge["target"]) if str(edge["target"]).isdigit() else edge["target"]
    G.add_edge(src, tgt, **edge)

# Compute layout (prefer graphviz top-down 'dot' layout, otherwise spring)
try:
    pos = nx.nx_agraph.graphviz_layout(G, prog='dot')
except Exception:
    pos = nx.spring_layout(G, seed=42)

# Prepare node and edge coordinates + hover text
node_x, node_y, node_text, node_ids = [], [], [], []
for node_id, (x, y) in pos.items():
    node = G.nodes[node_id]
    node_x.append(x)
    node_y.append(y)  # no flip: keep top-down from graphviz
    # Compose node hover text
    node_text.append(
        f"ID: {node_id}<br>"
        f"State: {node.get('state')}<br>"
        f"Value: {node.get('value')}<br>"
        f"Terminal: {node.get('is_terminal')}"
    )
    node_ids.append(node_id)

edge_x, edge_y = [], []
edge_hover_x, edge_hover_y, edge_text, edge_ids = [], [], [], []
for (u, v, attr) in G.edges(data=True):
    x0, y0 = pos[u]
    x1, y1 = pos[v]
    edge_x += [x0, x1, None]
    edge_y += [y0, y1, None]
    edge_hover_x.append((x0 + x1) / 2)
    edge_hover_y.append((y0 + y1) / 2)
    e_id = f"{u}-{v}"
    edge_ids.append(e_id)
    edge_text.append(
        f"Edge {u}→{v}<br>"
        f"Action: {attr.get('action')}<br>"
        f"Visits: {attr.get('visits')}<br>"
        f"Q: {float(attr.get('q', 0)):.3f}<br>"
        f"Prior: {float(attr.get('prior', 0)):.6f}"
    )

# Create Plotly Figure
fig = go.Figure()

# Edges trace (lines)
fig.add_trace(go.Scatter(
    x=edge_x, y=edge_y,
    mode='lines',
    line=dict(width=1, color='#888'),
    hoverinfo='none',
    name='edges'
))

# Invisible markers at midpoints for edge hover & clicks
fig.add_trace(go.Scatter(
    x=edge_hover_x, y=edge_hover_y,
    mode='markers',
    marker=dict(size=18, color='rgba(0,0,0,0)'),  # fully transparent marker
    hoverinfo='text',
    hovertext=edge_text,
    customdata=edge_ids,
    name='edge-hover'
))

# Node colors: terminal vs non-terminal
node_colors = [
    'tomato' if G.nodes[n].get('is_terminal') else 'lightblue'
    for n in node_ids
]

# Nodes trace
fig.add_trace(go.Scatter(
    x=node_x, y=node_y,
    mode='markers+text',
    marker=dict(
        size=22,
        color=node_colors,
        line=dict(width=2, color='darkblue'),
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
    yaxis=dict(visible=False),
    dragmode='zoom',         # make drag a zoom by default
    uirevision='mcts-graph'  # keep zoom/pan when figure updates
)

# Dash app
app = Dash(__name__, external_stylesheets=[dbc.themes.BOOTSTRAP])
app.layout = dbc.Container([
    html.H3("Monte Carlo Search Tree Visualization"),
    dcc.Graph(
        id='mcts-graph',
        figure=fig,
        style={'height': '800px'},
        config={
            'scrollZoom': True,       # enable mouse-wheel zooming
            'displayModeBar': True,
            'doubleClick': 'reset'
        }
    ),
    # store pinned annotations (list of dicts)
    dcc.Store(id='pinned-annotations', data=[])
], fluid=True)


@app.callback(
    Output('mcts-graph', 'figure'),
    Output('pinned-annotations', 'data'),
    Input('mcts-graph', 'clickData'),
    State('mcts-graph', 'figure'),
    State('pinned-annotations', 'data'),
)
def update_annotations(clickData, fig_dict, pinned_annotations):
    """
    Toggle pin/unpin annotation when user clicks a node or an edge midpoint.
    pinned_annotations is a list of dicts: {key, x, y, text}
    """
    # initialize pinned_annotations if None
    if pinned_annotations is None:
        pinned_annotations = []

    if not clickData:
        # no click -> just return current figure & annotations
        return fig_dict, pinned_annotations

    # Build a working figure
    fig_work = go.Figure(fig_dict)

    # Safely extract the clicked point (Plotly clickData has points list)
    point = clickData.get('points', [None])[0]
    if point is None:
        return fig_dict, pinned_annotations

    # Extract customdata (Plotly may wrap customdata as list inside point)
    customdata = point.get('customdata')
    # If customdata appears as list (from scatter with list), pick first element
    if isinstance(customdata, (list, tuple)) and len(customdata) == 1:
        customdata = customdata[0]

    # Coordinates where to place annotation (x, y)
    x = point.get('x')
    y = point.get('y')

    # Determine whether click is node or edge by customdata pattern
    key = None
    annotation_text = ""
    try:
        # If it's an edge id like '3-7'
        if isinstance(customdata, str) and "-" in customdata:
            u_str, v_str = customdata.split('-', 1)
            u = int(u_str) if u_str.isdigit() else u_str
            v = int(v_str) if v_str.isdigit() else v_str
            edge = G[u][v]
            annotation_text = (
                f"Edge {u}→{v}<br>"
                f"Action: {edge.get('action')}<br>"
                f"Visits: {edge.get('visits')}<br>"
                f"Q: {float(edge.get('q', 0)):.3f}<br>"
                f"Prior: {float(edge.get('prior', 0)):.6f}"
            )
            key = f"Edge-{u}-{v}"
        else:
            # treat as node id (could be int or string)
            node_id = int(customdata) if isinstance(customdata, (int, str)) and str(customdata).isdigit() else customdata
            node = G.nodes[node_id]
            annotation_text = (
                f"Node {node_id}<br>"
                f"State: {node.get('state')}<br>"
                f"Value: {node.get('value')}<br>"
                f"Terminal: {node.get('is_terminal')}"
            )
            key = f"Node-{node_id}"
    except Exception as e:
        # fallback: do not change annotations if we can't interpret the click
        print("Warning: failed to parse click customdata:", customdata, "error:", e)
        return fig_dict, pinned_annotations

    # Toggle pin/unpin
    existing_keys = [ann.get('key') for ann in pinned_annotations]
    if key in existing_keys:
        # unpin: remove
        pinned_annotations = [ann for ann in pinned_annotations if ann.get('key') != key]
    else:
        # pin: append
        pinned_annotations.append({
            'key': key,
            'x': x,
            'y': y,
            'text': annotation_text
        })

    # Rebuild annotation objects for layout
    annotations_layout = []
    for ann in pinned_annotations:
        annotations_layout.append(dict(
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
            opacity=0.95
        ))

    fig_work.update_layout(annotations=annotations_layout)

    return fig_work.to_dict(), pinned_annotations


if __name__ == '__main__':
    # Run Dash server
    app.run(debug=True)
