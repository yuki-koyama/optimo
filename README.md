# OptiMo

OptiMo is an "optimization-guided motion editing" system for authoring 3D character animations. OptiMo allows animators to effectively utilize the power of numerical optimization while keeping appropriate control.

![](docs/system.png)

## Project Web Page

<http://koyama.xyz/project/optimo/>

## Dependencies

### Not included

- Eigen <http://eigen.tuxfamily.org/>
- Qt (5.6 or higher) <http://doc.qt.io/qt-5/>
- OpenGL (2.1 or higher) <https://www.opengl.org/>
- glm <https://glm.g-truc.net/>

### Included via gitsubmodule

- NLopt <https://nlopt.readthedocs.io/>
- json11 <https://github.com/dropbox/json11>
- tinycolormap <https://github.com/yuki-koyama/tinycolormap>
- nlopt-util <https://github.com/yuki-koyama/nlopt-util>
- three-dim-util <https://github.com/yuki-koyama/three-dim-util>
- parallel-util <https://github.com/yuki-koyama/parallel-util>

### Included directly

- RBDL <https://bitbucket.org/rbdl/rbdl/> (their source codes for `v2.5.0` are included in this repository under the zlib license)

## Compilation Instruction

CMake <https://cmake.org/> is used for managing source codes. OptiMo can be built by, for example, 
```
git clone https://github.com/yuki-koyama/optimo.git --recursive
cd optimo
mkdir build
cd build
cmake ../
make
```

Note that the necessary third-party libraries (i.e., Eigen and Qt) should be installed before building OptiMo. If you use macOS and `brew`, you can easily install them by
```
brew install eigen qt
```

In some environments, you might need to specify `CMAKE_PREFIX_PATH` adequately so that CMake can find Qt5.

## Known Issues (Need Help!)

OptiMo is currently tested on macOS only. It is possible that OptiMo could not be built or run with other platforms such as Windows or Linux. Pull requests are welcome.

## Publication

Yuki Koyama and Masataka Goto. 2018. OptiMo: Optimization-Guided Motion Editing for Keyframe Character Animation. In Proceedings of 2018 CHI Conference on Human Factors in Computing Systems (CHI '18), pp.161:1--161:12. DOI: <https://doi.org/10.1145/3173574.3173735>

## Licensing

OptiMo is dual-licensed; You may use OptiMo under either *LGPLv3* or *our commercial license*. See the `LICENSE` files for details.

## Contributing

Pull requests are highly welcome. Please be aware that any contribution to this repository will be licensed under the above license condition.

## Authors

- Yuki Koyama
- Masataka Goto

## Copyright

Copyright (c) 2018 National Institute of Advanced Industrial Science and Technology (AIST) - <koyama.y@aist.go.jp>
