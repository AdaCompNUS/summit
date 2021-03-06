# Example command to convert an OSM network into a SUMO network.
# For left-handed road networks, append the flag --lefthand to the back.
# For lane width, SUMMIT assumes a lane width of 4.0m.

netconvert --osm-files ../Data/meskel_square.osm -o ../Data/meskel_square.net.xml --proj "+proj=merc +a=6378137 +b=6378137 +lat_ts=0.0 +lon_0=0.0 +x_0=0.0 +y_0=0 +k=1.0 +units=m +nadgrids=@null +wktext +no_defs" --geometry.remove --ramps.guess --edges.join --junctions.join --keep-edges.by-type highway.motorway,highway.motorway_link,highway.trunk,highway.trunk_link,highway.primary,highway.primary_link,highway.secondary,highway.secondary_link,highway.tertiary,highway.tertiary_link,highway.unclassified,highway.residential --no-turnarounds.except-deadend --tls.discard-loaded --tls.discard-simple --default.lanewidth 4.0
