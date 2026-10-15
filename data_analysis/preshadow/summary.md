# preshadow  (`session_20261015_091320`)

- Analysed window: 09:14:20-09:35:41 only

- Rover 57936, 1281 s, drove 151 m, 1.7-41.0 m from the BS
- BS at map (+34.5, +9.5) m, cell col 162 row 118 (matches 100% of the logged predictions)
- Link up 100% of the time in 3 contact(s); 1239 rover beacons heard at the BS (0.97/s)
- Measured RSSI -96 / -64 / -34 dBm (min / median / max)

## Delivery

| | |
|---|---|
| Data bundles delivered | 0 (rover generated 0) |
| Telemetry delivered | 1502 |
| Rover tx / retx | 0 / 0 |
| Data rate | 0.00 bundles/s |
| Latency | no data bundles |
| Delivered < 2 s / >= 2 s | 0 / 0 |
| Generated while link down | 0 |

## Predicted vs measured RSSI

Prediction = map path gain + a constant offset (median of measured - predicted).

| Model | n | offset [dB] | r | RMSE [dB] | MAE [dB] | slope |
|---|---|---|---|---|---|---|
| nn_pmnet | 1239 | -28.7 | 0.79 | 7.3 | 6.0 | 1.28 |
| nn | 1239 | -27.5 | 0.81 | 7.5 | 6.4 | 1.54 |
| avg | 1239 | -28.3 | 0.80 | 7.4 | 6.2 | 1.40 |

![radio maps](map_rm.png)

![comms](map_comms.png)

![time series](timeseries.png)

![pred vs true](pred_vs_true.png)

![link quality](link_quality.png)
