osm=$1

path=$2

osrm-extract ${path}/${osm}-latest.osm.pbf
osrm-partition ${path}/${osm}-latest.osrm
osrm-customize ${path}/${osm}-latest.osrm