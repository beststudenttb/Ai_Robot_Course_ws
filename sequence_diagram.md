# ROS2 Mecanum Patrol — 时序图

## 1. 正常巡逻 (Mission A→B→C)

```mermaid
sequenceDiagram
    participant M as Mission
    participant D as Decision
    participant P as Perception
    participant MD as MecanumDriver
    participant W as Webots

    Note over M: sequence = [A_point, B_point, C_point]

    loop 每个周期 (~32ms)
        P->>W: Supervisor.step()
        W-->>P: tracker position/orientation
        P->>D: /tracker_pose (PoseStamped)
        P-->>W: tf world→tracker
    end

    M->>D: Action Goal {target_name: "A_point"}
    D->>D: 查 named dict → A_point = (14.0, 0.0)
    D-->>M: ACCEPT

    loop 10Hz 直到到达
        D->>D: compute(vx, vy, omega)
        D->>MD: /cmd_vel (Twist)
        D->>P: /target_marker_state "A_point:green"
        D-->>M: Feedback {distance, cmd_vx, cmd_vy}
        MD->>MD: inverse_kinematics → wheel speeds
        MD->>W: 4× Motor.setVelocity()
    end

    D-->>M: Result {success: true, "arrived"}
    M->>M: idx+1, 发下一个 B_point
```

## 2. 键盘抢占

```mermaid
sequenceDiagram
    participant K as Keyboard
    participant M as Mission
    participant D as Decision
    participant P as Perception

    Note over M: 正在执行 A_point

    M->>D: Goal {target_name: "A_point"}
    D-->>M: ACCEPT
    Note over D: execute A_point...

    K->>M: /target_command "B_point"
    Note over M: on_keyboard: keyboard_name = "B_point"

    Note over M: spin 检测到 keyboard_name, break

    M->>D: Goal {target_name: "B_point"}
    Note over D: on_goal: ACCEPT (抢占)
    Note over D: 旧 handle.is_active = False
    Note over D: execute B_point...

    D-->>M: Result B: arrived
    Note over M: keyboard任务完成, 不推进idx
    Note over M: 回到被打断的 A_point
    M->>D: Goal {target_name: "A_point"}
```

## 3. Reset / Pause

```mermaid
sequenceDiagram
    participant K as Keyboard
    participant P as Perception
    participant MD as MecanumDriver
    participant W as Webots

    K->>P: /reset "r"
    P->>P: tracker.getField("translation").setSFVec3f([0,0,0])
    P->>P: 颜色全白

    K->>MD: /pause "p"
    MD->>MD: paused = not paused
    MD->>MD: 电机 setVelocity(0.0)
    Note over MD: on_timer: "paused"

    K->>MD: /pause "p" (再次)
    MD->>MD: paused = False
    Note over MD: 恢复接收 cmd_vel
```

## 4. Service 状态查询

```mermaid
sequenceDiagram
    participant CLI
    participant L as Logger
    participant P as Perception
    participant D as Decision

    CLI->>L: Service /get_status
    L->>P: Service /get_pose
    P-->>L: {x, y, yaw}
    L->>D: Service /get_target
    D-->>L: {cmd_vx, cmd_vy, target_name}
    L-->>CLI: {pose_x, pose_y, cmd_vx, cmd_vy, target_name}
```

## 5. 通信总览

```
                     ┌──────────────┐
                     │   Keyboard   │
                     └──┬───┬───┬──┘
          /target_command│   │   │ /reset, /pause
                     ┌───┘   │   └──────────┐
                     ▼       │              ▼
                 ┌───────┐   │    ┌──────────────┐
                 │Mission│   │    │  Perception  │◄─── Supervisor ── Webots
                 └───┬───┘   │    └──────┬───────┘
       Action        │       │           │ /tracker_pose
    /navigate_to     │       │           │ /tracker_state
                     ▼       │           ▼
                 ┌──────────┴───────────┐
                 │      Decision        │
                 └──────────┬──────────┘
                            │ /cmd_vel
                            │ /target_marker_state
                            ▼
                 ┌──────────────────────┐
                 │   MecanumDriver     │──► Webots motors
                 └──────────────────────┘
                        ▲
            /pause      │
                 ┌──────┘
                 │ Keyboard (r/p)
                 └──────────────────────┘
                        │
            /reset      ▼
                 ┌──────────────┐
                 │  Perception  │──► teleport
                 └──────────────┘

    Topic:
      /target_command      Keyboard → Mission
      /tracker_pose        Perception → Decision
      /tracker_state       Perception → MecanumDriver
      /cmd_vel             Decision → MecanumDriver
      /target_marker_state Decision → Perception
      /reset               Keyboard → Perception
      /pause               Keyboard → MecanumDriver

    Action:
      /navigate_to         Mission → Decision

    Service:
      /get_status          Logger (call /get_pose + /get_target)
      /get_pose            Perception
      /get_target          Decision
```
