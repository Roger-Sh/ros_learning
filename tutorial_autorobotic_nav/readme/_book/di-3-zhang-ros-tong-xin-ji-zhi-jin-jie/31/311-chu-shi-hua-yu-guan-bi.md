### 3.1.1 初始化

---

#### C++

##### 初始化

```cpp
/** @brief ROS初始化函数。
 *
 * 该函数可以解析并使用节点启动时传入的参数(通过参数设置节点名称、命名空间...) 
 *
 * 该函数有多个重载版本，如果使用NodeHandle建议调用该版本。 
 *
 * \param argc 参数个数
 * \param argv 参数列表
 * \param name 节点名称，需要保证其唯一性，不允许包含命名空间
 * \param options 节点启动选项，被封装进了ros::init_options
 *
 */
void init(int &argc, char **argv, const std::string& name, uint32_t options = 0);
```

---

#### Python

##### 初始化

```py
def init_node(name, argv=None, anonymous=False, log_level=None, disable_rostime=False, disable_rosout=False, disable_signals=False, xmlrpc_port=0, tcpros_port=0):
    """
    在ROS msater中注册节点

    @param name: 节点名称，必须保证节点名称唯一，节点名称中不能使用命名空间(不能包含 '/')
    @type  name: str

    @param anonymous: 取值为 true 时，为节点名称后缀随机编号
    @type anonymous: bool
    """
```



