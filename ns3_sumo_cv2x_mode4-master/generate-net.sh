SUMO_HOME="/usr/local/share/sumo/"

SAVE_DIR="traffic_files"

NET_FILE=${SAVE_DIR}/"net.xml"
FLOW_FILE=${SAVE_DIR}/"flow.xml"
ROUTE_FILE=${SAVE_DIR}/"route.xml"
REROUTER_FILE=${SAVE_DIR}/"reroute.xml"
netgenerate -g --grid.x-number 3 --grid.y-number 3 --grid.x-length 250 --grid.y-length 433 --default.speed 20.0 --o ${NET_FILE}
python3 ${SUMO_HOME}tools/randomTrips.py -n ${NET_FILE} -o ${FLOW_FILE} --begin 0 --end 1 --flows 10 --jtrrouter --trip-attribute 'departPos="random" departSpeed="max"'
jtrrouter --route-files=${FLOW_FILE} --net-file=${NET_FILE} --output-file=${ROUTE_FILE} --accept-all-destinations --allow-loops
python3 ${SUMO_HOME}tools/generateContinuousRerouters.py -n ${NET_FILE} -o ${REROUTER_FILE}

SIM_FILE=${SAVE_DIR}/"sim.sumocfg"
cat >${SIM_FILE} <<EOF
<configuration>
  <input>
    <net-file value="${NET_FILE#*/}"/>
    <route-files value="${ROUTE_FILE#*/}"/>
    <additional-files value="${REROUTER_FILE#*/}"/>
  </input>
</configuration>
EOF
