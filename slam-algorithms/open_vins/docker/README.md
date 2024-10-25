
- Как собрать образ?

    ```
    SLAM/slam-algorithms/open_vins/docker$ ./build.bash
    ```

- Как запустить контейнер?

    ```
    SLAM/slam-algorithms/open_vins/docker$ ./run_container.bash <abs path to ros2 bag>
    ```

    Тогда указанный датасет примонтируется в /home/workspace/colcon_ws_ov/data/

- Еще один терминал
    ```
    SLAM/slam-algorithms/open_vins/docker$ ./exec.bash    
    ```

- Как проиграть датасет?

    Должен быть собран образ 
    
    Должен быть запущен контейнер 
    
    Терминал 1
    ```
    /home/workspace/colcon_ws_ov$ source ./install/setup.bash
    /home/workspace/colcon_ws_ov$ ros2 launch ov_msckf subscribe.launch.py config:=euroc_mav
    ```
    
    Терминал 2
    ```
    /home/workspace/colcon_ws_ov$ source ./install/setup.bash
    /home/workspace/colcon_ws_ov$ ros2 run rviz2 rviz2 -d src/open_vins/ov_msckf/launch/display_ros2.rviz
    ```
        
    Терминал 3
    ```
    /home/workspace/colcon_ws_ov$ source ./install/setup.bash
    /home/workspace/colcon_ws_ov$ ros2 bag play <path to your dataset>
    ```


