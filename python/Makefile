LCM = lcm-gen
LCMTYPES_NAMES := $(shell cd ../lcmtypes && ls *.lcm)
LCMTYPES_PY := $(LCMTYPES_NAMES:%.lcm=%.py)
ALL = $(LCMTYPES_PY)

all: $(ALL)

%.py:../lcmtypes/%.lcm
	$(LCM) -p --package-prefix lcmtypes $<

clean:
	rm -rf lcmtypes/
