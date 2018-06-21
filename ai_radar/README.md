# Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.

@page dwx_radar_replay_sample Radar Point Clouds Sample

The Radar Replay sample demonstrates how to connect to a Radar and displays the generated point cloud in 3D.

For a list of currently supported Radar devices, see the <em>Release Notes</em>.

#### Point Cloud Dispay
![Radar Point Clouds Sample](sample_radar_replay.png)


## Prerequisites

- The Radar must be up and running, and connected to the network.

## Running the Sample

The command line for the sample to display live radar point clouds is:

    ./sample_radar_replay --ip=[radar IP address] --port=[lidar port] --device=[type of device]

Where [type of device] is one of the following:
- CONTINENTAL_ARS430
- DELPHI_ESR2_5

The command line for the sample to display recorded Radar point clouds is:

    ./sample_radar_replay --file=[radar bin file]

- The Radar file can be obtained with the provided recording tools.
- If no arguments are passed, a default Radar file is loaded.

## Output

The sample opens an X window to display a 3D point cloud. Beside points the output contains directed unit velocity vectors.
If velocity is 100 km/h or less then the directed vectors are red. If greater or equal to 200 then they are green.
Worldspace axes: Red-OX, Blue-OY, Green-OZ.

Use the mouse and keyboard to interact with the visualization:

- Mouse left button: rotate the point cloud
- Mouse wheel: zoom in or out
- SPACE: make a pause
- R: reset camera view and also reset artificially increased or decreased frame rate
- G: show/hide circular and rectangular grid
- F1 show/hide text messages hints
