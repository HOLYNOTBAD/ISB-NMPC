/*
 * Local qpOASES configuration for the isb_nmpc package.
 *
 * These values mirror the ACADO generated SO3 solver interface.
 */

#ifndef SO3_QPOASES_INTERFACE_HPP
#define SO3_QPOASES_INTERFACE_HPP

/** Maximum number of optimization variables. */
#ifndef QPOASES_NVMAX
#define QPOASES_NVMAX 140
#endif

/** Maximum number of constraints. */
#ifndef QPOASES_NCMAX
#define QPOASES_NCMAX 340
#endif

/** Maximum number of working set recalculations. */
#ifndef QPOASES_NWSRMAX
#define QPOASES_NWSRMAX 1440
#endif

/** Print level for qpOASES. */
#ifndef QPOASES_PRINTLEVEL
#define QPOASES_PRINTLEVEL PL_NONE
#endif

/** The value of EPS. */
#ifndef QPOASES_EPS
#define QPOASES_EPS 1.193e-07
#endif

/** Internally used floating point type. */
typedef float real_t;

#endif  // SO3_QPOASES_INTERFACE_HPP
