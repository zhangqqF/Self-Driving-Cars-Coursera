# Self-Driving-Cars-Coursera

> zhangqq  
>
> Mar-15, 2023
>
> Chongqing

---



Self-Driving Cars specialization of the University of Toronto on Coursera.

## Part 1 - Introduction to self-driving cars

### Final Project

#### [Installation]()

- Download the modified Carla package provided on Coursera.

  - Run the following command to test.

    ```
    CarlaUE4.exe -windowed -carla-no-networking
    ```

    - It remind you to install something, just follow the prompt.
    - If you open the UE succeed, use [ASDW]() keys control the car to run and turn, and [P]() to toggle to Auto-pilot mode, [Q]() to toggle reverse and forward driving mode.

  - Continue to test with the following commands

    ```
    CarlaUE4.exe /Game/Maps/RaceTrack -windowed -carla-no-networking -benchmark -fps=20
    ```

    ```
    CarlaUE4.exe /Game/Maps/RaceTrack -windowed -carla-server -benchmark -fps=20
    ```

- Install python 3.6.x

  - If you just used VPN，it would be possible error about Proxy，you can turn off the [use a proxy server]() and turn on to continue.

  - Install packages

    ```cmd
    pip install -r ...\requirements.txt
    ```

    the [requirements.txt]() is included the Carla package.

    Notice, 

#### [Do the Project]()

- Download the [Course1FinalProject]() package provided Coursera and place it to the [PythonClient]() folder that is under the Carla.

