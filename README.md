### Dynamic Gap

Insert GIF

### Installation

This branch is designed to be deployed in [Arena-Rosnav](https://arena-rosnav.readthedocs.io/en/latest/).

We forked our own version of the benchmarking environment to solve some pre-existent problems in the repositories. To install our version, enter the following command in the terminal

```curl https://raw.githubusercontent.com/max-assel/arena-rosnav/patch/install.sh | bash```

Then, open a new terminal and run the following command

```curl https://raw.githubusercontent.com/max-assel/arena-rosnav/patch/install2.sh | bash```

Finally, open another new terminal and run the following command

```curl https://raw.githubusercontent.com/max-assel/arena-rosnav/patch/install3_training.sh | bash```

### Testing

You must be in the poetry environment to run benchmarking, to do so

```cd ~/arena_ws/src/arena/arena-rosnav```

```poetry shell```