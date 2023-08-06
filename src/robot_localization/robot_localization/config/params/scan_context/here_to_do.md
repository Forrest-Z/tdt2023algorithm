按照GeYao README中的方法，重新生成基于自己基环境protobuf的proto：

打开lidar_localization/config/scan_context文件夹，输入如下命令，生成pb文件

```
protoc --cpp_out=./ key_frames.proto
protoc --cpp_out=./ ring_keys.proto
protoc --cpp_out=./ scan_contexts.proto
mv key_frames.pb.cc key_frames.pb.cpp
mv ring_keys.pb.cc ring_keys.pb.cpp
mv scan_contexts.pb.cc scan_contexts.pb.cpp
```
执行后会生成
```
key_frames.pb.hpp
ring_keys.pb.hpp
scan_contexts.pb.hpp

key_frames.pb.cpp
ring_keys.pb.cpp
scan_contexts.pb.cpp
```
之后，用以上步骤生成的的.pb.h文件替换 robot_localization/include/models/scan_context_manager/
用.pb.cpp替换 robot_localization/src/models/scan_context_manager/
中的三个同名文件（注意：需要剪切，确保config文件中新生成的文件都转移到对应目录下，config下没有h以及cpp，不能重复出现）

然后：
```c++
//将key_frames.pb.cpp中第4行
#include "key_frames.pb.h"
//改为
#include "../../../include/models/scan_context_manager/key_frames.pb.h"
//其余两个如是



```