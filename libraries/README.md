# Clone External Libraries here, including MuJoCo

```console
// Choose a MuJoCo release version to your own taste, but the version must be newer than 3.0.0.
$ git clone -b [tag name] https://github.com/deepmind/mujoco.git

// If you want to use modified MuJoCo, then just fork the original MuJoCo,
// , modify there and add it to the submodule
$ git submodule add [url: own git repo]
```

