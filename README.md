# Basic usage

  source .venv/bin/activate
## Stream data
python -m leg.io.stream


## plot realtime data
python -m leg.plots.stream_plot


## Realtime inference
python -m leg.models.realtime_inference 
python -m leg.models.gait_csv_sender





## send training commands
python -m estimulos.test

## Guardar data
python -m leg.io.store_stream

## train
python -m leg.models.train