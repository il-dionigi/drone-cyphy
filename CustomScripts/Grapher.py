import plotly.plotly as py
import plotly.tools as pyt
import plotly.graph_objs as go
import plotly.figure_factory as FF
pyt.set_credentials_file(username='scifi-drone', api_key='jMZp7ui3i8EVBjzt7qFQ')

import sys

import numpy as np
import pandas as pd

inFile = sys.argv[1]

df = pd.read_csv(inFile)

trace1 = go.Scatter(
                    x=df['time'], y=df['x_pos'], # Data
                    mode='lines', name='position x' # Additional options
                   )
trace2 = go.Scatter(
                    x=df['time'], y=df['y_pos'], # Data
                    mode='lines', name='position y' # Additional options
                   )
trace3 = go.Scatter(
                    x=df['time'], y=df['z_pos'], # Data
                    mode='lines', name='position z' # Additional options
                   )
layout = go.Layout(title='Plot of drone pos vs time',
                   plot_bgcolor='rgb(230, 230,230)')

fig = go.Figure(data=[trace1, trace2, trace3], layout=layout)

# Plot data in the notebook
py.iplot(fig, filename='drone_posVtime')
