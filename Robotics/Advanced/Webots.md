>Цель: Установка пакета webots_ros2 и запуск примеров моделирования на Windows.

Пакет webots_ros2 предоставляет интерфейс между ROS 2 и Webots. Он включает несколько подпакетов, включая webots_ros2_driver, который позволяет узлам ROS взаимодействовать с Webots. Другие подпакеты в основном являются примерами, которые показывают несколько возможных реализаций с использованием интерфейса.

Рекомендуется понимать основные принципы ROS, изложенные в разделе [[Начинающий|начинающий]]. В частности, [[Создание рабочего пространства|создание рабочего пространства]] и [[Создание пакета|создание пакета]] являются полезными предварительными условиями.

[[Webots]] является предварительным условием для использования пакета webots_ros2. Вы можете следовать процедуре установки или собрать его из исходников.

В качестве альтернативы вы также можете позволить webots_ros2 автоматически загружать Webots. Эта опция появляется, когда вы запускаете пример пакета и не обнаруживаете установку самого Webots.

Можно установить Webots в фоновом режиме из консоли администратора DOS, введя:

```
webots-R2023b_setup.exe /SUPPRESSMSGBOXES /VERYSILENT /NOCANCEL /NORESTART /ALLUSERS
```

Дополнительные параметры настройки доступны и задокументированы здесь:

```
webots-R2023b_setup.exe /?
```

Если после установки вы наблюдаете аномалии 3D-рендеринга или Webots дает сбой, настоятельно рекомендуется обновить графический драйвер.

Если у вас установлено более одной версии Webots, ROS 2 будет искать Webots в следующих местах (в этом порядке):

1) Если задана переменная среды ROS2_WEBOTS_HOME, ROS 2 будет использовать Webots в этой папке, независимо от ее версии.

2) Если задана переменная среды WEBOTS_HOME, ROS 2 будет использовать Webots в этой папке, независимо от ее версии.

3) Если ни один из предыдущих пунктов не задан/не установлен, ROS 2 будет искать Webots в путях установки по умолчанию для совместимой версии: C:\Program Files\Webots.

4) Если Webots не удалось найти, webots_ros2 покажет окно и предложит автоматическую установку Webots последней совместимой версии.

Windows WSL (подсистема Windows для Linux) улучшает пользовательский опыт с ROS 2 по сравнению с собственной установкой Windows, поскольку она работает на платформе Linux. Потребуется установка WSL с версией Ubuntu, совместимой с нашим дистрибутивом ROS, и обновление её до WSL2, следуя [официальному руководству Microsoft](https://learn.microsoft.com/en-us/windows/wsl/install).

#### Установка webots_ros2

Затем мы можем либо установить webots_ros2 из официально выпущенного пакета, либо установить его из последних актуальных релизов с [Github](https://github.com/cyberbotics/webots_ros2).

Следующие команды должны быть запущены внутри среды WSL.

```
sudo apt-get install ros-jazzy-webots-ros2
```

#### Запуск webots_ros2_universal_robot

WSL не поддерживает аппаратное ускорение (пока). Поэтому Webots следует запускать в Windows, пока часть ROS работает внутри WSL. Для этого необходимо выполнить следующие команды внутри среды WSL.

Сначала запустите среду ROS 2, если это еще не сделано.

```
source /opt/ros/jazzy/setup.bash
```

Установка переменной среды `WEBOTS_HOME` позволяет нам запустить определенную установку Webots (например, `C:\Program Files\Webots`). Используйте точку монтирования «/mnt» для ссылки на путь в собственной Windows.

```
export WEBOTS_HOME=/mnt/c/Program\ Files/Webots
```

Используйте команду запуска ROS 2 для запуска демонстрационных пакетов (например, `webots_ros2_universal_robot`).

```
ros2 launch webots_ros2_universal_robot multirobot_launch.py
```

Роботом Tiago можно управлять с помощью:

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

С более старыми версиями WSL RViz2 может не работать напрямую, так как дисплей недоступен. Чтобы использовать RViz, нужно обновить WSL:

В Windows Shell:
```
wsl --update
```

#### Настройка симуляции (базовая)

Организуем код в пользовательский пакет ROS 2. Создаем новый пакет с именем my_package из папки src нашего рабочего пространства ROS 2. Изменим текущий каталог нашего терминала на ros2_ws/src и запустим:

```
ros2 pkg create --build-type ament_cmake --license Apache-2.0 --node-name MyRobotDriver my_package --dependencies rclcpp geometry_msgs webots_ros2_driver pluginlib
```

Параметр `--node-name MyRobotDriver` создаст шаблон плагина `MyRobotDriver.cpp` C++ в подпапке `my_package/src`, который мы изменим позже. Параметр `--dependencies rclcpp geometry_msgs webots_ros2_driver pluginlib` указывает пакеты, необходимые плагину `MyRobotDriver` в файле `package.xml`.

Добавим папку `launch`, `worlds` и `resource` в папку `my_package`.

```
cd my_package
mkdir launch
mkdir worlds
mkdir resource
```

В итоге у нас должна получиться следующая структура папок:

```
src/
└── my_package/
    ├── launch/
    ├── my_package/
    │   ├── __init__.py
    │   └── my_robot_driver.py
    ├── resource/
    │   └── my_package
    ├── test/
    │   ├── test_copyright.py
    │   ├── test_flake8.py
    │   └── test_pep257.py
    ├── worlds/
    ├── package.xml
    ├── setup.cfg
    └── setup.py

```

Нам потребуется [файл](https://docs.ros.org/en/jazzy/_downloads/5ad123fc6a8f1ea79553d5039728084a/my_world.wbt)мира для запуска симуляции, его нужно переместить в `my_package/worlds/`.

На самом деле это довольно простой текстовый файл, который вы можете визуализировать в текстовом редакторе. Простой робот уже включен в этот файл мира `my_world.wbt`.

Для создания собственной модели робота в Webots, можно ознакомиться с этим [руководством](https://cyberbotics.com/doc/guide/tutorial-6-4-wheels-robot).

Подпакет `webots_ros2_driver` автоматически создает интерфейс ROS 2 для большинства датчиков. В этой задаче мы расширим этотинтерфейс, создав свой собственный пользовательский плагин. Этот пользовательский плагин является узлом ROS, эквивалентным контроллеру робота. Мы можем использовать его для доступа к [API](https://cyberbotics.com/doc/reference/robot?tab-language=python) робота Webots и создания собственных тем и сервисов для управления роботом.

Откроем `my_package/include/my_package/MyRobotDriver.hpp` в любимом редакторе и заменим его содержимое следующим:

```
#ifndef WEBOTS_ROS2_PLUGIN_EXAMPLE_HPP
#define WEBOTS_ROS2_PLUGIN_EXAMPLE_HPP

#include "rclcpp/macros.hpp"
#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace my_robot_driver {
class MyRobotDriver : public webots_ros2_driver::PluginInterface {
public:
  void step() override;
  void init(webots_ros2_driver::WebotsNode *node,
            std::unordered_map<std::string, std::string> &parameters) override;

private:

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      cmd_vel_subscription_;
  geometry_msgs::msg::Twist cmd_vel_msg;

  WbDeviceTag right_motor;
  WbDeviceTag left_motor;
};
} // namespace my_robot_driver
#endif
```

Определен класс `MyRobotDriver`, который наследуется от класса `webots_ros2_driver::PluginInterface`. Плагин должен переопределить функции `step(...)` и `init(...)`. Более подробная информация приведена в файле `MyRobotDriver.cpp`. Несколько вспомогательных методов, обратных вызовов и переменных-членов, которые будут использоваться внутри плагина, объявлены закрытыми.

Затем откроем `my_package/src/MyRobotDriver.cpp` в редакторе и замените его содержимое следующим:

```
#include "my_package/MyRobotDriver.hpp"

#include "rclcpp/rclcpp.hpp"
#include <cstdio>
#include <functional>
#include <webots/motor.h>
#include <webots/robot.h>

#define HALF_DISTANCE_BETWEEN_WHEELS 0.045
#define WHEEL_RADIUS 0.025

namespace my_robot_driver {
void MyRobotDriver::init(
    webots_ros2_driver::WebotsNode *node,
    std::unordered_map<std::string, std::string> &parameters) {

  right_motor = wb_robot_get_device("right wheel motor");
  left_motor = wb_robot_get_device("left wheel motor");

  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);

  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(right_motor, 0.0);

  cmd_vel_subscription_ = node->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::SensorDataQoS().reliable(),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg){
        this->cmd_vel_msg.linear = msg->linear;
        this->cmd_vel_msg.angular = msg->angular;
      }
  );
}

void MyRobotDriver::step() {
  auto forward_speed = cmd_vel_msg.linear.x;
  auto angular_speed = cmd_vel_msg.angular.z;

  auto command_motor_left =
      (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) /
      WHEEL_RADIUS;
  auto command_motor_right =
      (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) /
      WHEEL_RADIUS;

  wb_motor_set_velocity(left_motor, command_motor_left);
  wb_motor_set_velocity(right_motor, command_motor_right);
}
} // namespace my_robot_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_robot_driver::MyRobotDriver,
                       webots_ros2_driver::PluginInterface)
```

Метод `MyRobotDriver::init` выполняется после загрузки плагина пакетом `webots_ros2_driver`. Он принимает два аргумента:

* Указатель на `WebotsNode`, определяемый `webots_ros2_driver`, который позволяет получить доступ к функциям узла ROS 2.

* Аргумент `параметров` — это неупорядоченная карта строк, созданная из XML-тегов, указанных в файлах URDF (`my_robot.urdf`), и позволяет передавать параметры контроллеру. Он не используется в этом примере.

Он инициализирует плагин, настраивая моторы робота, устанавливая их положения и скорости и подписываясь на тему `/cmd_vel`.

```
void MyRobotDriver::init(
    webots_ros2_driver::WebotsNode *node,
    std::unordered_map<std::string, std::string> &parameters) {

  right_motor = wb_robot_get_device("right wheel motor");
  left_motor = wb_robot_get_device("left wheel motor");

  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);

  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(right_motor, 0.0);

  cmd_vel_subscription_ = node->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::SensorDataQoS().reliable(),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg){
        this->cmd_vel_msg.linear = msg->linear;
        this->cmd_vel_msg.angular = msg->angular;
      }
  );
}
```

Обратный вызов подписки объявлен как лямбда-функция, которая будет вызываться для каждого сообщения Twist, полученного по теме `/cmd_vel`, и сохранять его в переменной-члене `cmd_vel_msg`.

```
      [this](const geometry_msgs::msg::Twist::SharedPtr msg){
        this->cmd_vel_msg.linear = msg->linear;
        this->cmd_vel_msg.angular = msg->angular;
      }
```

Метод `step()` вызывается на каждом временном шаге моделирования. На каждом временном шаге метод извлекает желаемые `forward_speed` и `angular_speed` из `cmd_vel_msg`. Поскольку двигатели управляются угловыми скоростями, метод затем преобразует `forward_speed` и `angular_speed` в отдельные команды для каждого колеса. Это преобразование зависит от структуры робота, а точнее от радиуса колеса и расстояния между ними.

```
void MyRobotDriver::step() {
  auto forward_speed = cmd_vel_msg.linear.x;
  auto angular_speed = cmd_vel_msg.angular.z;

  auto command_motor_left =
      (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) /
      WHEEL_RADIUS;
  auto command_motor_right =
      (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) /
      WHEEL_RADIUS;

  wb_motor_set_velocity(left_motor, command_motor_left);
  wb_motor_set_velocity(right_motor, command_motor_right);
}
```

Последние строки файла определяют конец пространства имен `my_robot_driver` и включают макрос для экспорта класса `MyRobotDriver` как плагина с использованием макроса `PLUGINLIB_EXPORT_CLASS`. Это позволяет загружать плагин драйвером Webots ROS2 во время выполнения.

```
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_robot_driver::MyRobotDriver,
                       webots_ros2_driver::PluginInterface)
```

Хотя плагин реализован на языке C++, для взаимодействия с библиотекой контроллера Webots необходимо использовать API языка C.

#### Создание файла `my_robot_urdf`

Теперь создадим файл URDF для объявления плагина `MyRobotDriver`. Это позволит узлу ROS `webots_ros2_driver` запустить плагин и подключить его к целевому роботу.

В папке `my_package/resource` создаем текстовый файл с именем `my_robot.urdf` со следующим содержимым:

```
<?xml version="1.0" ?>
<robot name="My robot">
    <webots>
        <plugin type="my_robot_driver::MyRobotDriver" />
    </webots>
</robot>
```

#### Создание файла запуска

Давайте создадим файл запуска, чтобы легко запустить симуляцию и контроллер ROS одной командой. В папке `my_package/launch` создайте новый текстовый файл с именем `robot_launch.py` ​​со следующим кодом:

```
import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

def generate_launch_description():
    package_dir = get_package_share_directory('my_package')
    robot_description_path = os.path.join(package_dir, 'resource', 'my_robot.urdf')

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'my_world.wbt')
    )

    my_robot_driver = WebotsController(
        robot_name='my_robot',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )

    return LaunchDescription([
        webots,
        my_robot_driver,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
```

Объект `WebotsLauncher` — это пользовательское действие, позволяющее запустить экземпляр симуляции Webots. В конструкторе необходимо указать, какой файл мира будет открыт симулятором.

```
webots = WebotsLauncher(
    world=os.path.join(package_dir, 'worlds', 'my_world.wbt')
)
```

Затем создается узел ROS, взаимодействующий с имитируемым роботом. Этот узел, называемый `WebotsController`, находится в пакете `webots_ros2_driver`.

Узел (в WSL) сможет взаимодействовать с имитируемым роботом (в Webots на собственной платформе Windows) через TCP-соединение.

В нашем случае следует запустить один экземпляр этого узла, потому что у нас есть один робот в симуляции. Но если бы у нас было больше роботов в симуляции, нам пришлось бы запустить один экземпляр этого узла на каждого робота. Параметр `robot_name` используется для определения имени робота, к которому должен подключиться драйвер. Параметр `robot_description` содержит путь к файлу URDF, который ссылается на плагин `MyRobotDriver`. Вы можете видеть узел `WebotsController` как интерфейс, который подключает плагин контроллера к целевому роботу.

```
my_robot_driver = WebotsController(
    robot_name='my_robot',
    parameters=[
        {'robot_description': robot_description_path},
    ]
)
```

После этого два узла настраиваются на запуск в конструкторе `LaunchDescription`:

```
return LaunchDescription([
    webots,
    my_robot_driver,
```

Наконец, добавлена ​​необязательная часть для отключения всех узлов после завершения работы Webots (например, при его закрытии из графического пользовательского интерфейса).

```
launch.actions.RegisterEventHandler(
    event_handler=launch.event_handlers.OnProcessExit(
        target_action=webots,
        on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
    )
)
```

#### Редактирование дополнительных файлов

Прежде чем запустить файл запуска, необходимо изменить файлы `CMakeLists.txt` и `my_robot_driver.xml`:

* `CMakeLists.txt` определяет правила компиляции вашего плагина.
* `my_robot_driver.xml` необходим для `pluginlib`, чтобы найти плагин Webots ROS 2.

Откроем `my_package/my_robot_driver.xml` и заменим его содержимое на:

```
<library path="my_package">
  <!-- The `type` attribute is a reference to the plugin class. -->
  <!-- The `base_class_type` attribute is always `webots_ros2_driver::PluginInterface`. -->
  <class type="my_robot_driver::MyRobotDriver" base_class_type="webots_ros2_driver::PluginInterface">
    <description>
      This is a Webots ROS 2 plugin example
    </description>
  </class>
</library>
```

Откроем `my_package/CMakeLists.txt` и заменим его содержимое на:

```
cmake_minimum_required(VERSION 3.5)
project(my_package)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Besides the package specific dependencies we also need the `pluginlib` and `webots_ros2_driver`
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(webots_ros2_driver REQUIRED)

# Export the plugin configuration file
pluginlib_export_plugin_description_file(webots_ros2_driver my_robot_driver.xml)

# MyRobotDriver library
add_library(
  ${PROJECT_NAME}
  SHARED
  src/MyRobotDriver.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)
ament_target_dependencies(
  ${PROJECT_NAME}
  pluginlib
  rclcpp
  webots_ros2_driver
)
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# Install additional directories.
install(DIRECTORY
  launch
  resource
  worlds
  DESTINATION share/${PROJECT_NAME}/
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_package()
```

`CMakeLists.txt` экспортирует файл конфигурации плагина с помощью `pluginlib_export_plugin_description_file()`, определяет общую библиотеку плагина C++ `src/MyRobotDriver.cpp` и устанавливает зависимости include и библиотеки с помощью `ament_target_dependencies()`.

Затем файл устанавливает библиотеку, каталоги `launch`, `resource` и `worlds` в каталог `share/my_package`. Наконец, он экспортирует каталоги `include` и библиотеки с помощью `ament_export_include_directories()` и `ament_export_libraries()` соответственно и объявляет пакет с помощью `ament_package()`.

#### Тестирование кода

Из терминала в рабочей области WSL ROS 2 выполним:

```
colcon build
export WEBOTS_HOME=/mnt/c/Program\ Files/Webots
source install/local_setup.bash
ros2 launch my_package robot_launch.py
```

Обязательно используем префикс `/mnt` перед путем к папке установки Webots для доступа к файловой системе Windows из WSL.

Это запустит симуляцию. Webots будет автоматически установлен при первом запуске, если он еще не был установлен.

Затем откроем второй терминал и отправим команду:

```
ros2 topic pub /cmd_vel geometry_msgs/Twist  "linear: { x: 0.1 }"
```

Теперь робот движется вперед. На этом этапе робот способен слепо следовать нашим командам моторики. Но в конце концов он упрется в стену, когда мы прикажем ему двигаться вперед.

Закроем окно Webots, это также должно закрыть узлы ROS, запущенные из лаунчера. Закроем команду темы с помощью Ctrl+C во втором терминале.

#### Настройка симуляции (продвинутая)

Целью является реализация узла ROS 2, который обходит препятствия с помощью датчиков расстояния робота. Уделим основное внимание использованию устройств робота с интерфейсом `webots_ros2_driver`.

#### Обновление `my_robot.urdf`

Как упоминалось ранее, `webots_ros2_driver` содержит плагины для прямого сопряжения большинства устройств Webots с ROS 2. Эти плагины можно загрузить с помощью тега `<device>` в файле URDF робота. Атрибут `reference` должен соответствовать параметру имени устройства Webots. Список всех существующих интерфейсов и соответствующих параметров можно найти на странице [справки по устройствам](https://github.com/cyberbotics/webots_ros2/wiki/References-Devices). Для доступных устройств, которые не настроены в файле URDF, интерфейс будет создан автоматически, а для параметров ROS (например, частота обновления, имя темы и имя фрейма) будут использоваться значения по умолчанию.

В `my_robot.urdf` заменим все содержимое на:

```
<?xml version="1.0" ?>
<robot name="My robot">
    <webots>
        <device reference="ds0" type="DistanceSensor">
            <ros>
                <topicName>/left_sensor</topicName>
                <alwaysOn>true</alwaysOn>
            </ros>
        </device>
        <device reference="ds1" type="DistanceSensor">
            <ros>
                <topicName>/right_sensor</topicName>
                <alwaysOn>true</alwaysOn>
            </ros>
        </device>
        <plugin type="my_robot_driver::MyRobotDriver" />
    </webots>
</robot>
```

В дополнение к пользовательскому плагину, драйвер `webots_ros2_driver` будет анализировать теги `<device>`, ссылающиеся на узлы **DistanceSensor**, и использовать стандартные параметры в тегах `<ros>` для включения датчиков и присвоения им названий.

#### Создание узла ROS для избежания препятствий

Робот будет использовать стандартный узел ROS для обнаружения стены и отправки команд двигателям для ее избегания. В папке `my_package/include/my_package` создадим файл заголовка с именем `ObstacleAvoider.hpp` со следующим кодом:

```
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

class ObstacleAvoider : public rclcpp::Node {
public:
  explicit ObstacleAvoider();

private:
  void leftSensorCallback(const sensor_msgs::msg::Range::SharedPtr msg);
  void rightSensorCallback(const sensor_msgs::msg::Range::SharedPtr msg);

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr left_sensor_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr right_sensor_sub_;

  double left_sensor_value{0.0};
  double right_sensor_value{0.0};
};
```

В папке `my_package/src` создайте исходный файл с именем `ObstacleAvoider.cpp` со следующим кодом:

```
#include "my_package/ObstacleAvoider.hpp"

#define MAX_RANGE 0.15

ObstacleAvoider::ObstacleAvoider() : Node("obstacle_avoider") {
  publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

  left_sensor_sub_ = create_subscription<sensor_msgs::msg::Range>(
      "/left_sensor", 1,
      [this](const sensor_msgs::msg::Range::SharedPtr msg){
        return this->leftSensorCallback(msg);
      }
  );

  right_sensor_sub_ = create_subscription<sensor_msgs::msg::Range>(
      "/right_sensor", 1,
      [this](const sensor_msgs::msg::Range::SharedPtr msg){
        return this->rightSensorCallback(msg);
      }
  );
}

void ObstacleAvoider::leftSensorCallback(
    const sensor_msgs::msg::Range::SharedPtr msg) {
  left_sensor_value = msg->range;
}

void ObstacleAvoider::rightSensorCallback(
    const sensor_msgs::msg::Range::SharedPtr msg) {
  right_sensor_value = msg->range;

  auto command_message = std::make_unique<geometry_msgs::msg::Twist>();

  command_message->linear.x = 0.1;

  if (left_sensor_value < 0.9 * MAX_RANGE ||
      right_sensor_value < 0.9 * MAX_RANGE) {
    command_message->angular.z = -2.0;
  }

  publisher_->publish(std::move(command_message));
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto avoider = std::make_shared<ObstacleAvoider>();
  rclcpp::spin(avoider);
  rclcpp::shutdown();
  return 0;
}
```

Этот узел создаст издателя для команды и подпишется на темы датчиков здесь:

```
 publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

  left_sensor_sub_ = create_subscription<sensor_msgs::msg::Range>(
      "/left_sensor", 1,
      [this](const sensor_msgs::msg::Range::SharedPtr msg){
        return this->leftSensorCallback(msg);
      }
  );

  right_sensor_sub_ = create_subscription<sensor_msgs::msg::Range>(
      "/right_sensor", 1,
      [this](const sensor_msgs::msg::Range::SharedPtr msg){
        return this->rightSensorCallback(msg);
      }
  );
```

При получении измерения от левого датчика оно будет скопировано в поле участника:

```
void ObstacleAvoider::leftSensorCallback(
    const sensor_msgs::msg::Range::SharedPtr msg) {
  left_sensor_value = msg->range;
}
```

Наконец, сообщение будет отправлено в тему `/cmd_vel`, когда будет получено измерение от правого датчика. `Command_message` зарегистрирует как минимум скорость движения вперед в `linear.x`, чтобы заставить робота двигаться, когда препятствие не обнаружено. Если любой из двух датчиков обнаружит препятствие, `command_message` также зарегистрирует скорость вращения в `angular.z`, чтобы заставить робота повернуть направо.

```
void ObstacleAvoider::rightSensorCallback(
    const sensor_msgs::msg::Range::SharedPtr msg) {
  right_sensor_value = msg->range;

  auto command_message = std::make_unique<geometry_msgs::msg::Twist>();

  command_message->linear.x = 0.1;

  if (left_sensor_value < 0.9 * MAX_RANGE ||
      right_sensor_value < 0.9 * MAX_RANGE) {
    command_message->angular.z = -2.0;
  }

  publisher_->publish(std::move(command_message));
}
```

#### Обновление дополнительных файлов

Отредактируем `CMakeLists.txt` и добавим компиляцию и установку` obstacle_avoider`:

```
cmake_minimum_required(VERSION 3.5)
project(my_package)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Besides the package specific dependencies we also need the `pluginlib` and `webots_ros2_driver`
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(webots_ros2_driver REQUIRED)

# Export the plugin configuration file
pluginlib_export_plugin_description_file(webots_ros2_driver my_robot_driver.xml)

# Obstacle avoider
include_directories(
  include
)
add_executable(obstacle_avoider
  src/ObstacleAvoider.cpp
)
ament_target_dependencies(obstacle_avoider
  rclcpp
  geometry_msgs
  sensor_msgs
)
install(TARGETS
  obstacle_avoider
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)

# MyRobotDriver library
add_library(
  ${PROJECT_NAME}
  SHARED
  src/MyRobotDriver.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)
ament_target_dependencies(
  ${PROJECT_NAME}
  pluginlib
  rclcpp
  webots_ros2_driver
)
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# Install additional directories.
install(DIRECTORY
  launch
  resource
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_package()
```

Перейдем к файлу robot_launch.py ​​и заменим его на:

```
import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

def generate_launch_description():
    package_dir = get_package_share_directory('my_package')
    robot_description_path = os.path.join(package_dir, 'resource', 'my_robot.urdf')

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'my_world.wbt')
    )

    my_robot_driver = WebotsController(
        robot_name='my_robot',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )

    obstacle_avoider = Node(
        package='my_package',
        executable='obstacle_avoider',
    )

    return LaunchDescription([
        webots,
        my_robot_driver,
        obstacle_avoider,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
```

Это создаст узел `obstacle_avoider`, который будет включен в `LaunchDescription`.

#### Тест кода для избегания препятствий

Запустим симуляцию из терминала в рабочей области ROS 2:

Из терминала в рабочей области WSL ROS 2 выполним:

```
colcon build
export WEBOTS_HOME=/mnt/c/Program\ Files/Webots
source install/local_setup.bash
ros2 launch my_package robot_launch.py
```

Обязательно воспользуемся префиксом `/mnt` перед путем к папке установки Webots для доступа к файловой системе Windows из WSL.

Наш робот должен двигаться вперед и перед тем, как удариться о стену, он должен повернуться по часовой стрелке. Мы можем нажать `Ctrl+F10` в Webots или перейти в меню `View`, `Optional Rendering` и `Show DistanceSensor Rays`, чтобы отобразить диапазон датчиков расстояния робота.