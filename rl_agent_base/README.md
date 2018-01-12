rl_agent包, 用于完成于rl_impl中Agent采样样本的过程. 算法将优化后的控制器及其参数通过Topic的形式发送给ROS中的Agent, Agent解析Message, 恢复控制器, 使用给定的控制器在仿真模型或实际机器人在执行, 并记录样本, 执行完成后, 将样本发送给算法. 

rl_agent主要分两个阶段:
- Reset Phase
重置阶段, 接受到算法的Reset指令后, 按照给定的初始状态, 通过Reset控制器控制机器人到达初始状态, 并返回一个样本(空样本, 仅标识重置完成). 

- Trial Phase
尝试阶段, 接受到算法的Trial指令后, 按照给定的控制器以及控制器参数, 在机器人上进行实际执行, 并记录样本, 达到结束条件时(如失败, 达到尝试步长等), 返回一个样本, 样本包含全部执行过程, 并且, 样本的内容, 在gps.proto中进行定义.

rl_agent主要包含如下几个部分:
- `Controller`
控制器类, 以Plugin的形式存在, 单独编译成库`librl_agent_ctrl_lib.so`, 通过`class_loader`进行加载, 在parameter.yaml中通过参数的形式进行共性参数定义及加载. 主要完成通过不同的控制器参数, 恢复控制器, 提供接口, 输入当前状态X, 输出当前行为U. `Controller`作为基类, 定义了基本接口, 以及成员变量. 类似如下:
```cpp
	// 配置参数Map定义
    typedef boost::variant<bool, int, double, std::string,
          std::vector<std::string>, Eigen::MatrixXd, Eigen::VectorXd,
          std::vector<gps::SampleType>> CtrlConfigVar;
    typedef std::map<std::string, CtrlConfigVar> CtrlConfigMap;
    
    class Controller {
    protected:
      // 状态包含哪几种类型, 类型标识统一使用样本(Sample)类型标识
      std::vector<gps::SampleType> state_definition_;
    public:
      // 更新函数, Take an action. 输入X, 输出U
      virtual void update(const Eigen::VectorXd& states,
          ros::Time current_time, ros::Duration duration,
          Eigen::VectorXd& torques) = 0;
      // 是否结束
      virtual bool isFinished() const = 0;
      // 配置样本, 仅仅配置Action维度(对于不同的控制器, 或许会产生不同的行为)
      virtual void configureSample(boost::scoped_ptr<Sample>&) = 0;
      // reset()接口, 清除所有参数. 子类选择实现.
      virtual void reset(ros::Time update_time) { };
      // 配置函数, 通过给定的参数恢复控制器
      virtual void configure(CtrlConfigMap& options) { };
    };
```
`TrialController`也是一个抽象类(继承于`Controller`), 作为不同Trial控制器的基类. 定义类似如下:
```cpp
    class TrialController : public Controller {
    protected:
      int t_;
      int T_;
      bool is_configured_;
    public:
      virtual void getAction(int t,
          const Eigen::VectorXd& state, Eigen::VectorXd& action) = 0;

      virtual void configure(CtrlConfigMap& options) override;
      virtual bool isFinished() const override { return t_ > T_; }
      virtual bool isConfigured() const { return is_configured_; }
      int getStepCounter() {return t_; }
      int getTrialLength() {return T_; }
    };
```
`TrialController`在`Controller`的基础上, 针对Trial进行了再次封装, 提供了几个函数的基本实现, 并重定义了纯虚函数`getAction()`, 完成在不同时间步的输入到输出的映射.
控制器类虽然在使用上依赖于`Sample`提供输入状态, 但是在实现上自我封闭, 不依赖任何外部数据, 仅仅对外提供配置接口和工具函数`update()`或`getAction()`, 而函数的传入参数从何而来, 并不取决于控制器, 而是由控制器的所有者来完成.
在具体的实现上, 控制器的种类以及参数, 是由算法传递过来的参数决定(或许, 重置控制器种类只有一种, 但是目标状态必定是由算法传递得到). 所以, 控制器的配置, 均是由消息驱动, 当接收到具体消息时, 才进行控制器的配置.
- `Sensor`
`Sensor`类完成对机器人传感器的数据采集, 计算, 提供算法所有需要的原始数据(除Action外, 例如Joint position, velocity, effort, enviroment等数据). 为了解偶, 该类以组合模式的形式出现. 所有`Sensor`子类将会持有自己所计算的状态列表, 也是以`gps::SampleType`来标识, 不同的子类, 填充不同的类型的数据到`Sample`中. 
`Sensor`基类的定义如下所示:
```cpp
	// Sensor配置参数Map定义
    using DataHandleVariant =
        boost::variant<bool*, int*, double*, std::string,
          std::vector<std::string>, Eigen::MatrixXd, Eigen::VectorXd,
          hardware_interface::JointCommandInterface*>;
    typedef std::map<std::string, DataHandleVariant> DataHandleMap;

    class Sensor : public Composite<Sensor> {
    public:
      // 配置传感器接口, 部分参数通过配置参数Map传入, 部分参数通过ROS Parameter传入
      virtual void configureSensor(ros::NodeHandle& nh, DataHandleMap& handleMap);
      // 配置该传感器所采集的SampleType的MetaData
      virtual void configureSample(boost::scoped_ptr<Sample>& sample);
      // 更新传感器的数据
      virtual void updateSensor(const ros::Time& current, const ros::Duration& duration);
      // 更新样本, 若t使用默认参数, 则类似于push_back操作, 由样本自己计算填入位置
      virtual void updateSample(boost::scoped_ptr<Sample>& sample, int t = CURRENT_TIME_FLAG);
    };
```

`Sensor`类体系, 也独立编译成库`librl_agent_sensor_lib.so`, 由parameter.yaml文件配置ROS Parameter来完成传感器的初始化, 通过`class_loader`进行实例的创建. 针对不同的机器人, 需要配置不同的参数. 填入参数如下所示:
```yaml
    sensors:
      lib_path:
        /home/silence/Workspace/Rl/dragon_rl/dragon_ws/devel/lib/librl_agent_sensor_lib.so
      types: # list the class name of all sensor
        - ChainJointEncoder
        - JointEncoder
    #   - IMU
    #   - xxx
      configures: # configure all of sensor options
        ChainJointEncoder:
          names: # Optional
            - chain_joints_encoder
          chain_joints_encoder:
            chain: {root_link: torso, tip_link: left_gripper_base}
            joints:
              - left_s0
              - left_s1
              - left_e0
              - left_e1
              - left_w0
              - left_w1
              - left_w2
            encoder_filter_params:
              "0.29752 -106.5 -6447.3059 0.00029752 0.8935 -6.4473 1.4876e-07 0.00094675 0.99678; 0.70248 106.5 6447.3059"

        JointEncoder:
          names: # Optional
            - joint_encoders
          joint_encoders:
            joints:
              - left_s0
              - left_s1
              - left_e0
              - left_e1
              - left_w0
              - left_w1
              - left_w2
    #    IMU: # list the names of all imu
    #      - imu_1
    #      - imu_2
    #    xxx: # list the names of all xxx
    #      - xxx
```
- `Sample`
`Sample`类, 作为传感器数据的抽象, 也是最终所需要的结果. Reset的完成和Trial的结束都需要将`Sample`转换成Message传递给算法.
该类也作为`Sensor`和`Controller`之间的桥梁, `Controller`所需要的数据, 均是从`Sample`中获取, 通过获取的状态计算得到的Action又将填入`Sample`, 如此往复, 得到最终的样本轨迹$\tau = {(x_1, u_1), (x_2, u_2), ..., (x_N, u_N)}$. 所以在每个控制循环中, 必须先更新`Sensor`(将当前最新的状态填入到`Sample`中), 然后更新`Controller`(将当前状态对应执行的行为填入`Sample`中).
该类每一个样本类型`gps::SampleType`以Map(`SampleMap`)的形式存在. 定义如下:
```cpp
// typedef SampleMap
    typedef boost::variant<
        bool,   uint8_t,         std::vector<int>, int,
        double, Eigen::MatrixXd, Eigen::VectorXd
        > SampleTypeVariant;
    typedef std::vector<SampleTypeVariant> TypeSampleData;
    typedef OptionsMap AdditionalInfo;
    typedef struct {
      SampleDataFormat  data_format_;
      int               entries_size_;
      AdditionalInfo    additional_info_;
    } MetaData;
    typedef struct {
      TypeSampleData data_list_;
      MetaData       meta_data_;
    } TypeSample;
    typedef std::map<gps::SampleType, TypeSample> SampleMap;
```
每个键(`gps::SampleType`)对应的值是一个结构体(`TypeSample`), 该结构体由两部分构成, 其一(`TypeSampleData`)是真实数据, 其二(`MetaData`)是MetaData(描述数据的数据). 真实数据以`vector`的形式出现, 第i个元素是第i时间步对应的状态数据. 每一个状态数据的类型由`MetaData::SampleDataFormat`标识, `MetaData::entries_size_`标识一共有多少条数据.
除了上述数据组织结构外, `Sample`类还提供了数据类型格式配置, get/set接口等.
- `RobotPlugin`
`RobotPlugin`作为上述三个大类的持有者, 综合进行配置, 更新, 通信等任务. 主要功能有如下几点:
	- **与算法进行交互**
	通过ROS Pub/Sub通信模型与算法进行交互. 通过parameter.yaml文件指定不同通信通道的Topic名称.
	- **创建, 配置及销毁`Sensor`, `Controller`, `Sample`实例**
	在接收到对应的指令后, 分别解析Message配置相应的`Sensor`, `Controller`和初始化`Sample`. 并持有三类对象的实例, 负责创建, 使用, 以及最后销毁这三类对象的实例.
	- **实现样本采集的既定工作流程**
	虽然在实现中尽可能的独立每个模块, 但是相互之间难免仍会存在耦合关系, 例如, 在控制循环中, 必须首先更新`Sensor`, 使用`Sensor`填充`Sample`, 从`Sample`中获取当前状态, 传递给`Controller`得到Action, 然后执行Action, 并再次填充`Sample`. 该工作流程是既定的, 不能更换先后顺序. 以及什么时候执行Reset Controller什么时候执行Trial Controller. 控制器的切换等工作, 均需要由该类来完成. 所有的耦合关系都集中于该类中.

`RobotPlugin`类是消息驱动, 通过解析接收到算法的不同消息, 执行不同的任务.