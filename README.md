# X3cator Robot - Debugging Guide

## Service and Message Debugging

### Show Messages
To display all ROS messages:
```bash
./src/x3cator_msgs/scripts/show_msgs.sh
```

### Show Services
To display all available ROS services:
```bash
./src/x3cator_msgs/scripts/show_srvs.sh
```

## Gazebo Process Debugging

### Kill Gazebo Processes
If Gazebo is not launching properly or you encounter "Address already in use" errors, use these commands to clean up existing processes.


When running `ps aux | grep gz`, we got these processes:
```bash
ubuntu   1210503  0.0  0.0   2632   304 pts/1    T    01:36   0:00 /bin/sh -c gzclient   
ubuntu   1210507  0.1  0.0      0     0 pts/1    Z    01:36   0:05 [gzclient] <defunct>
ubuntu   1218865  0.0  0.0   2632   364 pts/1    T    01:41   0:00 /bin/sh -c gzclient   
ubuntu   1218869  0.3  0.0      0     0 pts/1    Z    01:41   0:11 [gzclient] <defunct>
ubuntu   1227535  0.0  0.0   2632   488 pts/1    T    01:44   0:00 /bin/sh -c gzclient   
ubuntu   1227539  0.9  0.0      0     0 pts/1    Z    01:44   0:25 [gzclient] <defunct>
ubuntu   1282522  0.0  0.0   2632   604 pts/1    T    02:17   0:00 /bin/sh -c gzclient   
ubuntu   1282526  1.7  0.0      0     0 pts/1    Z    02:17   0:13 [gzclient] <defunct>
ubuntu   1285349  0.0  0.0   2632   604 pts/1    T    02:18   0:00 /bin/sh -c gzserver
ubuntu   1285351  0.0  0.0   2632   544 pts/1    T    02:18   0:00 /bin/sh -c gzclient   
ubuntu   1285352 11.9  3.2 4183688 250104 pts/1  TLl  02:18   1:24 gzserver
ubuntu   1285354  2.7  3.1 3540376 242080 pts/1  TLl  02:18   0:19 gzclient
```

### Kill Commands Needed

1. Kill main Gazebo processes:
```bash
kill -9 1285352    # Kill active gzserver
kill -9 1285354    # Kill active gzclient
```

2. Kill parent shell processes:
```bash
kill -9 1210503 1218865 1227535 1282522 1285349 1285351
```

### Verification Steps
After running kill commands, verify:
```bash
# Check for remaining processes
ps aux | grep gz

# Check if Gazebo port is still in use
lsof -i :11345
```

## System Cleanup Commands

### Kill All Gazebo Processes
```bash
killall gzserver
killall gzclient
killall gz
```

### Check Port Usage
```bash
lsof -i :11345
```

### Find Zombie Processes
```bash
ps aux | grep gz
```
