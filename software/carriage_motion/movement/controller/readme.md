# PID controller design

## Analysis of input data
Data is read from the AddPU unit, which is sampling the number of ticks per
revolution of the motor shaft. Every motor has its ID (according to following
table) which is used to let the AddPU know, which data user want to read.

### Code identification
| Wheel | ID    |
| :---: | :---: |
| right | 0xb0  |
| left  | 0xc0  |

### Wheel parameters measurement
| Parameter | Value [mm] |
| :---:     | :---:      |
| diameter | 118         |
|perimeter | 370.7 (calculated)|

### Wheel parameters measurement (comprises wheel deformation)
| Parameter   | Value      |
| :---:       | :---:      |
| revolutions |  3         |
| distance    | 1086 [mm]  |
| ticks       | 2935       |
| primeter    | 362 [mm]   |
| wheel gauge | 212 [mm]   |  

