import pandas
import numpy

cartPkIDMap = {
    21:2173,
    22:2174,
    23:2175,
    24:2176,
    25:2177
}

# curtis measurments are microns
# ferrule offsets are microns

def xyFerruleOff(xyFerrule, xyFiber, xyPin, rot, scale, invert=False):
    # from image looking at face of fiber measurements
    # scale puts measurements into microns

    #first invert x values to get mirror image (looking)
    # from other side of the fiber
    xyFerrule = numpy.asarray(xyFerrule)*scale
    xyFiber = numpy.asarray(xyFiber)*scale
    xyPin = numpy.asarray(xyPin)*scale
    if invert:
        xyFerrule[0] = -1*xyFerrule[0]
        xyFiber[0] = -1*xyFiber[0]
        xyPin[0] = -1*xyPin[0]
    # reference everything from center of fiber
    xyFerrule = xyFerrule - xyFiber
    xyPin = xyPin - xyFiber
    vectorMag = numpy.linalg.norm(xyFerrule)
    f_theta = numpy.arctan2(xyFerrule[1],xyFerrule[0])
    p_theta = numpy.arctan2(xyPin[1],xyPin[0])
    if f_theta < 0:
        f_theta += 2*numpy.pi
    if p_theta < 0:
        p_theta += 2*numpy.pi
    # find angle from gx to ferrule offset vector phi
    phi = rot - p_theta + f_theta
    # get x component and y component the g camera frame
    # reverse the vector (to point from ferrule center to fiber center)
    x = -1*numpy.cos(phi)*vectorMag
    y = -1*numpy.sin(phi)*vectorMag

    return x,y

def curtisNew():
    meas = pandas.read_csv("Guide_Fiber_Offsets.csv")
    gprobes = pandas.read_csv("gprobe.csv")

    for ii,row in meas.iterrows():
        # Ferrule X,Y is offset from fiber center to ferrule
        # we need offset from Ferrule center to fiber
        # and we need to invert x guide camera sees other side
        # of fiber
        cart = int(row["Cart"])
        fiber = int(row["Fiber"])
        cartpk = cartPkIDMap[cart]
        gprobe = gprobes[(gprobes.gprobe_id==fiber) & (gprobes.cartridge_pk==cartpk)]
        xyFerrule = numpy.array([row["Ferrule X"],row["Ferrule Y"]])
        xyFiber = numpy.array([0,0])
        xyPin = numpy.array([row["Pin X"], row["Pin Y"]])
        scale = 1
        rot = numpy.radians(float(gprobe.rotation))
        x,y = xyFerruleOff(xyFerrule, xyFiber, xyPin, rot, scale)
        print("cart=%i, fiber=%i, xyFerruleOff=[%.4f, %.4f]"%(cart,fiber,x,y))

def curtisOld():
    meas = pandas.read_csv("Guide_Fiber_Offsets.csv")
    gprobes = pandas.read_csv("gprobe.csv")

    for ii,row in meas.iterrows():
        # Ferrule X,Y is offset from fiber center to ferrule
        # we need offset from Ferrule center to fiber
        # and we need to invert x guide camera sees other side
        # of fiber
        cart = int(row["Cart"])
        fiber = int(row["Fiber"])

        # invert x (looking from other side)
        fxc = -1*row["Ferrule X"]
        fyc = row["Ferrule Y"]
        pxc = -1*row["Pin X"]
        pyc = row["Pin Y"]

        cartpk = cartPkIDMap[cart]
        gprobe = gprobes[(gprobes.gprobe_id==fiber) & (gprobes.cartridge_pk==cartpk)]
        assert len(gprobe)==1
        rot = numpy.radians(float(gprobe.rotation))
        vectorMag = numpy.sqrt(fxc**2+fyc**2)
        fc_theta = numpy.arctan2(fyc,fxc)
        if fc_theta < 0:
            fc_theta += 2*numpy.pi
        pc_theta = numpy.arctan2(pyc,pxc)
        if pc_theta < 0:
            pc_theta += 2*numpy.pi
        # find angle from gx to ferrule offset vector phi
        phi = rot - pc_theta + fc_theta
        # get x component and y component the g camera frame
        # reverse the vector (to point from ferrule center to fiber center)
        x = -1*numpy.cos(phi)*vectorMag
        y = -1*numpy.sin(phi)*vectorMag

        print("cart=%i, fiber=%i, xyFerruleOff=[%.4f, %.4f]"%(cart,fiber,x,y))

def paul():
    meas = pandas.read_csv("paul.csv")
    gprobes = pandas.read_csv("gprobe.csv")
    cart = 21
    scale = 3.04 # microns per pixel
    for ii, row in meas.iterrows():
        fiber = int(row["fiberid"])
        cartpk = cartPkIDMap[cart]
        gprobe = gprobes[(gprobes.gprobe_id==fiber) & (gprobes.cartridge_pk==cartpk)]
        xyFerrule = numpy.array([row["xFerrule"],row["yFerrule"]])
        xyFiber = numpy.array([row["xFiber"], row["yFiber"]])
        xyPin = numpy.array([row["xPin"], row["yPin"]])
        rot = numpy.radians(float(gprobe.rotation))
        x,y = xyFerruleOff(xyFerrule, xyFiber, xyPin, rot, scale, invert=True)
        print("cart=%i, fiber=%i, xyFerruleOff=[%.4f, %.4f]"%(cart,fiber,x,y))

print("curtis")
curtisNew()

print("")
print("paul")
paul()



