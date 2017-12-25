AGENT_ROOT=..
PROTO_SRC_DIR=./proto
ALG_DIR=$AGENT_ROOT/rl_impl
DST_DIR=$ALG_DIR/build

# Hack to compile directly into src folders for now
CPP_OUT_DIR=$AGENT_ROOT/rl_agent_base/include/gps/proto
PROTO_BUILD_DIR=$DST_DIR/proto
PY_PROTO_BUILD_DIR=$ALG_DIR/python/gps/proto

mkdir -p "$PROTO_BUILD_DIR"
mkdir -p "$PY_PROTO_BUILD_DIR"
touch $PY_PROTO_BUILD_DIR/__init__.py

mkdir -p "$CPP_OUT_DIR"
protoc -I=$PROTO_SRC_DIR --cpp_out=$CPP_OUT_DIR $PROTO_SRC_DIR/gps.proto
protoc -I=$PROTO_SRC_DIR --python_out=$PY_PROTO_BUILD_DIR $PROTO_SRC_DIR/gps.proto

echo "Done"
