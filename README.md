# summit_agriculture

## Installation

```shell
git clone --recurse-submodules git@github.com:mgonzs13/summit_agriculture.git
rosdep install --from-paths src --ignore-src -r -y
```

## Usage

```shell
ros2 launch summit_cornfield cornfield.launch.py
```

```shell
ros2 launch summit_localization localization.launch.py use_sim_time:=True
```
