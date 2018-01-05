#!/bin/bash

select cck in "coffee" "charge" "kill"; do
    echo " "
    echo "Enter a command (coffee) or (charge) or (kill): "
    case $cck in
        coffee ) rostopic pub -r 10 /chatter std_msgs/String 'abcdefg'; break;;
        charge ) break;;
        kill ) exit;;
    esac
done
