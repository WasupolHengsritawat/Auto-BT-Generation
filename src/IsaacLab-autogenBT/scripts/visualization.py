import os
import json
import math
import networkx as nx
import plotly.graph_objects as go
from dash import Dash, html, dcc, Input, Output, State
import dash_bootstrap_components as dbc

# ---------- config ----------
script_dir = os.path.dirname(os.path.abspath(__file__))
logs_dir = os.path.abspath(os.path.join(script_dir, "..", "logs"))

date_time = "2025-11-28_16-24-57"
model_name = "rvnn_iter000"
count = 2

json_path = os.path.join(logs_dir, date_time, model_name, f"mcts_tree_{count}.json")
# ------------------------------------------------

# Load JSON
if not os.path.exists(json_path):
    raise FileNotFoundError(f"Could not find file: {json_path}")

with open(json_path, "r") as f:
    data = json.load(f)

# Build graph (node ids preserved as ints if numeric)
G = nx.DiGraph()
for node in data.get("nodes", []):
    node_id = int(node["id"]) if isinstance(node["id"], (int, str)) and str(node["id"]).isdigit() else node["id"]
    G.add_node(node_id, **node)

for edge in data.get("edges", []):
    src = int(edge["source"]) if str(edge["source"]).isdigit() else edge["source"]
    tgt = int(edge["target"]) if str(edge["target"]).isdigit() else edge["target"]
    G.add_edge(src, tgt, **edge)

# Compute layout (prefer graphviz dot if available)
try:
    pos = nx.nx_agraph.graphviz_layout(G, prog='dot')
except Exception:
    pos = nx.spring_layout(G, seed=42)

# Prepare arrays indexed by node order (node_ids keeps consistent ordering)
node_ids = list(pos.keys())
node_x = [pos[n][0] for n in node_ids]
node_y = [pos[n][1] for n in node_ids]
node_text = [
    f"ID: {n}<br>State: {G.nodes[n].get('state')}<br>Value: {G.nodes[n].get('value')}<br>Evaluated by {G.nodes[n].get('evaluated_bt')}<br>Terminal: {G.nodes[n].get('is_terminal')}"
    for n in node_ids
]

# Edge geometry / hover data
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

# Helpers to compute bounding box and padded ranges (for auto-zoom)
def compute_ranges_for_nodes(node_set, pad=0.15):
    xs = [pos[n][0] for n in node_set]
    ys = [pos[n][1] for n in node_set]
    if not xs or not ys:
        return None, None
    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    # Add padding relative to span
    x_span = max(1.0, x_max - x_min)
    y_span = max(1.0, y_max - y_min)
    x_pad = x_span * pad
    y_pad = y_span * pad
    return [x_min - x_pad, x_max + x_pad], [y_min - y_pad, y_max + y_pad]

# Function to build figure with view options:
def build_figure(highlight_ids=None, view_mode="whole", autozoom=False):
    """
    highlight_ids: set of node IDs to be marked as 'matched' (gold)
    view_mode: "whole" (show full tree), or "neighborhood" (show only matched + parents + children)
    autozoom: when True and view_mode is 'neighborhood', set axis ranges to fit nodes
    """
    if highlight_ids is None:
        highlight_ids = set()

    # Determine neighbor set if in neighborhood mode
    neighbor_set = set()
    if view_mode == "neighborhood" and highlight_ids:
        for n in highlight_ids:
            # add the node itself
            if n in G:
                neighbor_set.add(n)
                # parents (predecessors)
                for p in G.predecessors(n):
                    neighbor_set.add(p)
                # children (successors)
                for c in G.successors(n):
                    neighbor_set.add(c)

    # Build color and opacity for each node
    node_marker_colors = []
    node_marker_opacity = []
    for n in node_ids:
        is_terminal = bool(G.nodes[n].get('is_terminal'))
        if n in highlight_ids:
            node_marker_colors.append("gold")
            node_marker_opacity.append(1.0)
        elif view_mode == "neighborhood" and n in neighbor_set:
            # parents/children: use different pastel color depending on terminal
            node_marker_colors.append("lightsalmon" if is_terminal else "lightgreen")
            node_marker_opacity.append(1.0)
        elif view_mode == "neighborhood":
            # dim everything outside the neighborhood
            node_marker_colors.append("lightgray")
            node_marker_opacity.append(0.20)
        else:
            node_marker_colors.append("tomato" if is_terminal else "lightblue")
            node_marker_opacity.append(1.0)

    # Edges opacity: if neighborhood mode, keep edges only if both endpoints in neighbor_set (or involve matched node)
    edge_line_colors = []
    edge_line_opacity = []
    # we will recreate edge lists (x/y) selectively so easier to control visibility
    filtered_edge_x, filtered_edge_y = [], []
    filtered_edge_hover_x, filtered_edge_hover_y, filtered_edge_text, filtered_edge_ids = [], [], [], []

    for (u, v, attr) in G.edges(data=True):
        show_edge = True
        if view_mode == "neighborhood" and highlight_ids:
            # only show edges if both endpoints are in neighbor_set
            if not (u in neighbor_set and v in neighbor_set):
                show_edge = False

        if show_edge:
            x0, y0 = pos[u]
            x1, y1 = pos[v]
            filtered_edge_x += [x0, x1, None]
            filtered_edge_y += [y0, y1, None]
            filtered_edge_hover_x.append((x0 + x1) / 2)
            filtered_edge_hover_y.append((y0 + y1) / 2)
            e_id = f"{u}-{v}"
            filtered_edge_ids.append(e_id)
            filtered_edge_text.append(
                f"Edge {u}→{v}<br>"
                f"Action: {attr.get('action')}<br>"
                f"Visits: {attr.get('visits')}<br>"
                f"Q: {float(attr.get('q', 0)):.3f}<br>"
                f"Prior: {float(attr.get('prior', 0)):.6f}"
            )
            # edges for neighborhood are full opacity; otherwise default low-opacity line color
            edge_line_colors.append("#888")
            edge_line_opacity.append(1.0 if view_mode == "whole" or show_edge else 0.15)

    fig = go.Figure()

    # Edges trace (lines)
    fig.add_trace(go.Scatter(
        x=filtered_edge_x if filtered_edge_x else edge_x,
        y=filtered_edge_y if filtered_edge_y else edge_y,
        mode='lines',
        line=dict(width=1, color='#888'),
        hoverinfo='none',
        name='edges',
        opacity=1.0
    ))

    # Invisible markers for edge hover & clicks (only for shown edges)
    fig.add_trace(go.Scatter(
        x=filtered_edge_hover_x if filtered_edge_hover_x else edge_hover_x,
        y=filtered_edge_hover_y if filtered_edge_hover_y else edge_hover_y,
        mode='markers',
        marker=dict(size=18, color='rgba(0,0,0,0)'),
        hoverinfo='text',
        hovertext=filtered_edge_text if filtered_edge_text else edge_text,
        customdata=filtered_edge_ids if filtered_edge_ids else edge_ids,
        name='edge-hover'
    ))

    # Nodes trace
    fig.add_trace(go.Scatter(
        x=node_x, y=node_y,
        mode='markers+text',
        marker=dict(
            size=22,
            color=node_marker_colors,
            line=dict(width=2, color='darkblue'),
            opacity=node_marker_opacity
        ),
        text=[str(n) for n in node_ids],
        textposition="top center",
        hovertext=node_text,
        hoverinfo='text',
        customdata=node_ids,
        name='nodes'
    ))

    # Layout: uirevision so zoom persists when updating figure, dragmode zoom by default
    layout_kwargs = dict(
        showlegend=False,
        margin=dict(l=10, r=10, t=10, b=10),
        hovermode='closest',
        plot_bgcolor='white',
        paper_bgcolor='white',
        dragmode="zoom",
        xaxis=dict(visible=False),
        yaxis=dict(visible=False),
        uirevision="mcts-graph"
    )

    # If autozoom and neighborhood view, set axis ranges to bounding box of neighbor_set (if any)
    if autozoom and view_mode == "neighborhood" and highlight_ids:
        # bounding box uses neighbor_set; fallback to highlight_ids if neighbor_set empty
        use_nodes = neighbor_set if neighbor_set else highlight_ids
        x_range, y_range = compute_ranges_for_nodes(use_nodes, pad=0.20)
        if x_range and y_range:
            layout_kwargs['xaxis'] = dict(range=x_range, showgrid=False, zeroline=False, visible=False)
            layout_kwargs['yaxis'] = dict(range=y_range, showgrid=False, zeroline=False, visible=False)

    fig.update_layout(**layout_kwargs)

    return fig

# ----------------- DASH APP -----------------
app = Dash(__name__, external_stylesheets=[dbc.themes.BOOTSTRAP])

app.layout = dbc.Container([
    html.H3("Monte Carlo Search Tree Visualization"),

    dbc.Row([
        dbc.Col(
            dcc.Input(
                id="search-box",
                type="text",
                placeholder="Search node ID or state (e.g., 12, patrol, g01). Multiple: '12, patrol'",
                style={"width": "100%"}
            ),
            width=7
        ),
        dbc.Col(
            dbc.Button("Search", id="search-button", color="primary", style={"width": "100%"}),
            width=2
        ),
        dbc.Col(
            dcc.RadioItems(
                id="view-mode",
                options=[
                    {"label": "Whole tree", "value": "whole"},
                    {"label": "Node + parents & children", "value": "neighborhood"}
                ],
                value="whole",
                inline=True
            ),
            width=3
        )
    ], className="mb-3"),

    dcc.Graph(
        id='mcts-graph',
        figure=build_figure(),
        style={'height': '820px'},
        config={'scrollZoom': True, 'displayModeBar': True, 'doubleClick': 'reset'}
    ),

    # store pinned annotations
    dcc.Store(id='pinned-annotations', data=[])
], fluid=True)

# ------------- Search callback ----------------
@app.callback(
    Output("mcts-graph", "figure"),
    Input("search-button", "n_clicks"),
    State("search-box", "value"),
    State("view-mode", "value"),
    prevent_initial_call=True
)
def search_nodes(n_clicks, query, view_mode):
    """
    When user clicks Search:
     - parse query tokens
     - find matched node ids (by id or substring in state)
     - build figure with view_mode option
     - if view_mode == 'neighborhood' autozoom to fit the neighborhood
    """
    if not query or not str(query).strip():
        # empty query -> just return whole tree
        return build_figure(view_mode="whole")

    query = str(query).strip()
    tokens = [q.strip().lower() for q in query.split(",") if q.strip()]

    matched = set()
    for n in node_ids:
        node_state = str(G.nodes[n].get("state", "")).lower()
        for t in tokens:
            # exact numeric id match
            if t.isdigit() and int(t) == n:
                matched.add(n)
            # substring match in state
            elif t in node_state:
                matched.add(n)

    if not matched:
        # nothing found -> show whole tree unchanged
        return build_figure(view_mode="whole")

    # Build figure with selected view_mode; autozoom only for neighborhood
    return build_figure(highlight_ids=matched, view_mode=view_mode, autozoom=(view_mode == "neighborhood"))

# ------------- Annotation toggle (click) -------------
@app.callback(
    Output('mcts-graph', 'figure', allow_duplicate=True),
    Output('pinned-annotations', 'data'),
    Input('mcts-graph', 'clickData'),
    State('mcts-graph', 'figure'),
    State('pinned-annotations', 'data'),
    prevent_initial_call=True
)
def update_annotations(clickData, fig_dict, pinned_annotations):
    """
    Toggle pin/unpin annotation when user clicks a node or an edge midpoint.
    """
    if pinned_annotations is None:
        pinned_annotations = []

    if not clickData:
        return fig_dict, pinned_annotations

    fig = go.Figure(fig_dict)

    # Extract clicked point
    point = clickData.get('points', [None])[0]
    if point is None:
        return fig_dict, pinned_annotations

    customdata = point.get('customdata')
    # If customdata is a single-element list (sometimes), pick first element
    if isinstance(customdata, (list, tuple)) and len(customdata) == 1:
        customdata = customdata[0]

    x = point.get('x')
    y = point.get('y')

    key = None
    annotation_text = ""

    try:
        if isinstance(customdata, str) and "-" in customdata:
            u_str, v_str = customdata.split('-', 1)
            u = int(u_str) if u_str.isdigit() else u_str
            v = int(v_str) if v_str.isdigit() else v_str
            edge = G[u][v]
            annotation_text = (
                f"Edge {u}->{v}<br>"
                f"Action: {edge.get('action')}<br>"
                f"Visits: {edge.get('visits')}<br>"
                f"Q: {float(edge.get('q',0)):.3f}<br>"
                f"Prior: {float(edge.get('prior',0)):.6f}"
            )
            key = f"Edge-{u}-{v}"
        else:
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
        # ignore invalid clicks
        print("Warning: couldn't parse click customdata:", customdata, e)
        return fig_dict, pinned_annotations

    # Toggle pinned annotation
    existing_keys = [ann.get('key') for ann in pinned_annotations]
    if key in existing_keys:
        pinned_annotations = [ann for ann in pinned_annotations if ann.get('key') != key]
    else:
        pinned_annotations.append({'key': key, 'x': x, 'y': y, 'text': annotation_text})

    # Rebuild layout annotations
    annotations_layout = []
    for ann in pinned_annotations:
        annotations_layout.append(dict(
            x=ann['x'], y=ann['y'], xref="x", yref="y",
            text=ann['text'], showarrow=True, arrowhead=4,
            ax=20, ay=-20, bgcolor="white",
            bordercolor="black", borderwidth=1, opacity=0.95
        ))

    fig.update_layout(annotations=annotations_layout)

    return fig, pinned_annotations

# ---------- run ----------
if __name__ == '__main__':
    app.run(debug=True)
