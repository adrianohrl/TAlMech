BAG_FILE="2018-03-02-15-21-02.bag"
FOLDER="analytics"
mkdir -p ${FOLDER}

TOPIC="/analytics/cycles"
CSV_FILE="cycles.csv"
rostopic echo -b ${BAG_FILE} -p ${TOPIC} >> ${FOLDER}/${CSV_FILE}

TOPIC="/analytics/waypoints"
CSV_FILE="waypoints.csv"
rostopic echo -b ${BAG_FILE} -p ${TOPIC} >> ${FOLDER}/${CSV_FILE}

TOPIC="/analytics/waypoints/x"
CSV_FILE="waypoints-x.csv"
rostopic echo -b ${BAG_FILE} -p ${TOPIC} >> ${FOLDER}/${CSV_FILE}

TOPIC="/analytics/waypoints/y"
CSV_FILE="waypoints-y.csv"
rostopic echo -b ${BAG_FILE} -p ${TOPIC} >> ${FOLDER}/${CSV_FILE}

TOPIC="/analytics/skills/camera"
CSV_FILE="skills-camera.csv"
rostopic echo -b ${BAG_FILE} -p ${TOPIC} >> ${FOLDER}/${CSV_FILE}

TOPIC="/analytics/skills/battery"
CSV_FILE="skills-battery.csv"
rostopic echo -b ${BAG_FILE} -p ${TOPIC} >> ${FOLDER}/${CSV_FILE}

TOPIC="/analytics/skills/battery/level"
CSV_FILE="skills-level-battery.csv"
rostopic echo -b ${BAG_FILE} -p ${TOPIC} >> ${FOLDER}/${CSV_FILE}

TOPIC="/analytics/skills/strengh"
CSV_FILE="skills-strengh.csv"
rostopic echo -b ${BAG_FILE} -p ${TOPIC} >> ${FOLDER}/${CSV_FILE}

TOPIC="/analytics/skills/strengh/level"
CSV_FILE="skills-level-strengh.csv"
rostopic echo -b ${BAG_FILE} -p ${TOPIC} >> ${FOLDER}/${CSV_FILE}

TOPIC="/analytics/skills/processor"
CSV_FILE="skills-processor.csv"
rostopic echo -b ${BAG_FILE} -p ${TOPIC} >> ${FOLDER}/${CSV_FILE}

TOPIC="/analytics/skills/processor/level"
CSV_FILE="skills-level-processor.csv"
rostopic echo -b ${BAG_FILE} -p ${TOPIC} >> ${FOLDER}/${CSV_FILE}

TOPIC="/analytics/skills/laserscan"
CSV_FILE="skills-laserscan.csv"
rostopic echo -b ${BAG_FILE} -p ${TOPIC} >> ${FOLDER}/${CSV_FILE}

OCTAVE_EXPRESSION="bag_to_eps"
octave --silent --eval "${OCTAVE_EXPRESSION}"


