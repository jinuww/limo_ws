# py_bt_ros_limo_rescue

Behavior Tree(BT) 기반으로 ROS2(Action/Service/Topic)를 연결해 실행하는 프로젝트이다.
현재 구조는 `scenarios/limo_rescue/default_bt.xml` 에 정의된 BT를 로딩하여 실행한다.


------------------------------------------------------------
터미널 1:
  cd ~/py_bt_ros_limo_rescue/py_bt_ros
  python3 main.py --config config.yaml

- bt_visualiser.enabled: true 이면 pygame 창으로 BT 실행 상태가 표시된다.
- 실행 중:
  - q 또는 ESC : 종료
  - p          : 일시정지/재개

------------------------------------------------------------
상황 0) “Patrol로 들어가는지” 확인 (ExitClose=false, Detected=false)
목표

ExitClose가 false → ExitMode 진입

detected=false → PatrolSequence 실행(SetNextPoint→AlignToNextPoint→MoveToPoint)

0-1. 로봇 위치
ros2 topic pub -1 /bt/robot_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}
}"

0-2. 비상구는 멀리 (ExitClose=false)
ros2 topic pub -r 2 /bt/exit_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {position: {x: 10.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}
}"

0-3. 타겟 미검출(Detected=false)
ros2 topic pub -r 2 /yoloworld/target_detected std_msgs/msg/Bool "{data: false}"


✅ 이때 BT 화면에서 **PatrolSequence 쪽(AlignToNextPoint/MoveToPoint)**로 파란 RUNNING이 가면 성공.

상황 1) “Detected=true + TargetClose=false → ApproachToTarget 수행” 확인
목표

detected=true

타겟이 멀다(>0.8m) → AlignToTargetPoint + ApproachToTarget 실행

1-1. 로봇 위치(그대로)
ros2 topic pub -1 /bt/robot_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}
}"

1-2. 타겟 위치(멀리: 2.0m → TargetClose=false)
ros2 topic pub -r 2 /yoloworld/target_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}
}"

1-3. 타겟 검출(Detected=true)
ros2 topic pub -r 2 /yoloworld/target_detected std_msgs/msg/Bool "{data: true}"


✅ 이때 BT 화면에서 AlignToTargetPoint → ApproachToTarget로 RUNNING이 가면 성공.
(순찰은 멈추고 접근 루틴으로 “갈아타야” 정상)

상황 2) “Detected=true + TargetClose=true → 접근 스킵” 확인
목표

detected=true

타겟이 가깝다(<=0.8m) → ApproachSequence가 실행되지 않고 바로 다음 단계로 넘어감

2-1. 로봇 위치
ros2 topic pub -1 /bt/robot_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}
}"

2-2. 타겟 위치(가깝게: 0.3m → TargetClose=true)
ros2 topic pub -r 2 /yoloworld/target_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {position: {x: 0.3, y: 0.0, z: 0.0}, orientation: {w: 1.0}}
}"

2-3. detected=true 유지
ros2 topic pub -r 2 /yoloworld/target_detected std_msgs/msg/Bool "{data: true}"


✅ 이때 BT에서 ApproachToTarget이 회색(미실행)으로 남고, 바로 Exit 쪽(MakeRescueVoice/AlignToExitPoint/NavigateToExit)로 넘어가면 성공.

상황 3) “ExitClose=true면 즉시 종료” 확인 (가장 중요한 종료 데모)
목표

IsExitClose=true → 최상단에서 트리 종료(SUCCESS), ExitMode 실행 안 함

3-1. 로봇 위치
ros2 topic pub -1 /bt/robot_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}
}"

3-2. 비상구 위치를 아주 가깝게 (0.2m)
ros2 topic pub -r 2 /bt/exit_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {position: {x: 0.2, y: 0.0, z: 0.0}, orientation: {w: 1.0}}
}"


✅ 이때 BT 화면에서 **IsExitClose가 SUCCESS(보통 초록/성공 처리)**로 되고, ExitMode 아래 액션들이 실행 안 되면 성공.
