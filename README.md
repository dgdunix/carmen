# carmen

## Configure
<pre>
./configure --noWerror
</pre>
## make it
<pre>
make -j8
</pre>
## Run the simulator (Enter carmen-0.7.4-beta/bin)
<pre>
./central &
./param_daemon -r pioneer-I ../data/freiburg.map &
./simulator &
./robot &
./localize &
./navigator &
./robotgui &
./navigatorgui &
</pre>

## IPC test (Enter carmen/src/ipc)
<pre>
// make it
make
// start central to handle ipc , you can run on remote device
./central
// start test receiver 
./test_receive host-name
// start test generate ipc message
./test_generate remote_ip host_name
<pre>
