#!/bin/sh

[ "$(uci -q get system.@system[0].ttylogin)" = 0 ] && exec /bin/ash --login

exec /bin/login
