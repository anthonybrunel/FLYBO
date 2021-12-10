#init_dir
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"


mkdir -p $SCRIPT_DIR/init/maze/mesh && mkdir -p $SCRIPT_DIR/init/maze/log && mkdir -p $SCRIPT_DIR/init/maze/vol;
mkdir -p $SCRIPT_DIR/init/office_b/mesh && mkdir -p $SCRIPT_DIR/init/office_b/log && mkdir -p $SCRIPT_DIR/init/office_b/vol;
mkdir -p $SCRIPT_DIR/init/office_c/mesh && mkdir -p $SCRIPT_DIR/init/office_c/log && mkdir -p $SCRIPT_DIR/init/office_c/vol;
mkdir -p $SCRIPT_DIR/init/facility_a/mesh && mkdir -p $SCRIPT_DIR/init/facility_a/log && mkdir -p $SCRIPT_DIR/init/facility_a/vol;
mkdir -p $SCRIPT_DIR/init/facility_b/mesh && mkdir -p $SCRIPT_DIR/init/facility_b/log && mkdir -p $SCRIPT_DIR/init/facility_b/vol;
mkdir -p $SCRIPT_DIR/init/facility_c/mesh && mkdir -p $SCRIPT_DIR/init/facility_c/log && mkdir -p $SCRIPT_DIR/init/facility_c/vol;
mkdir -p $SCRIPT_DIR/init/platform_a/mesh && mkdir -p $SCRIPT_DIR/init/platform_a/log && mkdir -p $SCRIPT_DIR/init/platform_a/vol;
mkdir -p $SCRIPT_DIR/init/platform_b/mesh && mkdir -p $SCRIPT_DIR/init/platform_b/log && mkdir -p $SCRIPT_DIR/init/platform_b/vol;
mkdir -p $SCRIPT_DIR/init/warehouse/mesh && mkdir -p $SCRIPT_DIR/init/warehouse/log && mkdir -p $SCRIPT_DIR/init/warehouse/vol;
mkdir -p $SCRIPT_DIR/init/pp/mesh && mkdir -p $SCRIPT_DIR/init/pp/log && mkdir -p $SCRIPT_DIR/init/pp/vol;


#cp directory tree in each method
mkdir -p $SCRIPT_DIR/esm && cp -r "$SCRIPT_DIR/init/"* "$SCRIPT_DIR/esm/"
mkdir -p $SCRIPT_DIR/esm_recon && cp -r "$SCRIPT_DIR/init/"* "$SCRIPT_DIR/esm_recon/"
mkdir -p $SCRIPT_DIR/aep && cp -r "$SCRIPT_DIR/init/"* "$SCRIPT_DIR/aep/"
mkdir -p $SCRIPT_DIR/nbvp && cp -r "$SCRIPT_DIR/init/"* "$SCRIPT_DIR/nbvp/"
mkdir -p $SCRIPT_DIR/rapid && cp -r "$SCRIPT_DIR/init/"* "$SCRIPT_DIR/rapid/"
mkdir -p $SCRIPT_DIR/splatplanner && cp -r "$SCRIPT_DIR/init/"* "$SCRIPT_DIR/splatplanner/"
mkdir -p $SCRIPT_DIR/classic && cp -r "$SCRIPT_DIR/init/"* "$SCRIPT_DIR/classic/"


#gt directory
mkdir -p $SCRIPT_DIR/gt/maze
mkdir -p $SCRIPT_DIR/gt/office_b
mkdir -p $SCRIPT_DIR/gt/office_c
mkdir -p $SCRIPT_DIR/gt/facility_a
mkdir -p $SCRIPT_DIR/gt/facility_b
mkdir -p $SCRIPT_DIR/gt/facility_c
mkdir -p $SCRIPT_DIR/gt/platform_a
mkdir -p $SCRIPT_DIR/gt/platform_b
mkdir -p $SCRIPT_DIR/gt/warehouse
mkdir -p $SCRIPT_DIR/gt/pp
