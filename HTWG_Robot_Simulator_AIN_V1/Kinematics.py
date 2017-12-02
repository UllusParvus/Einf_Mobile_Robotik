

def curveDrive(v, r, delta_theta):


    n = 200
    omega = v/r
    tau = delta_theta/omega
    print('omega -> ' + str(omega))
    print('tau -> ' + str(tau))

    motions = [[v, omega] for i in range(n)]
    #radius = v/omega
    #print('radius -> ' + str(radius))

    return [tau/n, motions]


def straightDrive(v, l):
    n = 100

    t = l/v
    print('time -> ' + str(t))
    motions = [[v, 0.0] for i in range(n)]

    return [t/n, motions]