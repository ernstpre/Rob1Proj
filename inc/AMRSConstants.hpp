#ifndef AMRSCONSTANTS_HPP_
#define AMRSCONSTANTS_HPP_

namespace AMRSC
{
    namespace MOT
    {
    const double QMax = 2.5;                    // [N] 0.1 / 0.04
    const double qdMax = 0.848;                 // [m/s ]21.2 * 0.04
    const double i = 3441.0 / 104.0 / 0.04;     // [1/m]
    const double R = 8.0;                       // [hm]
    const double kM = 8.44e-3;                  // [Nm/A]
    }
    namespace CONT
    {
        const double D = 0.7;                   // [-]
        const double s = 2.2;                   // [-]
        const double M = 0.046;                 // [kg]
        const double ILIMIT = 0.1;              // [m]
    }
}

#endif // AMRSCONSTANTS_HPP_