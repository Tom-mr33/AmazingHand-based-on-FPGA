# ah_protocol 主机侧单元测试

不依赖 STM32/HAL，直接用 gcc 编译协议模块：

~~~sh
gcc -std=c11 -Wall -Wextra -Werror \
  -I ../Core/Inc \
  ../Core/Src/ah_protocol.c test_ah_protocol.c \
  -o test_ah_protocol
./test_ah_protocol
~~~

预期输出：

~~~
AH_PROTOCOL_TEST_OK
~~~

覆盖：CRC16 已知向量、命令/遥测编解码往返、分片解析、坏 CRC 丢弃、
DIRECT_TARGETS 解析。Windows 下 gcc 的临时目录若含中文路径导致汇编器报错，
可先把 TEMP/TMP 指到纯英文目录。
