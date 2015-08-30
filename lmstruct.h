/**
 * @file        lmstruct.h
 *
 * @brief       Declarations of parameter records, used in lmmin.h and lmcurve.h
 *
 * @details     **Library** lmfit (Levenberg-Marquardt least squares fitting)
 *
 * @author      Joachim Wuttke, Forschungszentrum Juelich GmbH (2004-2013)
 *
 * @license     **License** FreeBSD
 *
 * @see         [Homepage](http://apps.jcns.fz-juelich.de/lmfit)
 */

#ifndef LMSTRUCT_H
#define LMSTRUCT_H
#undef __BEGIN_DECLS
#undef __END_DECLS
#ifdef __cplusplus
# define __BEGIN_DECLS extern "C" {
# define __END_DECLS }
#else
# define __BEGIN_DECLS /* empty */
# define __END_DECLS /* empty */
#endif
__BEGIN_DECLS

#include <stdio.h>

/**
 * @brief   Collection of input parameters for fit control.
 */
typedef struct {
    /**
     * @brief   Relative error desired in the sum of squares.
     * @details Termination occurs when both the actual and
     *          predicted relative reductions in the sum of squares
     *          are at most ftol.
     */
    double ftol;
    /**
     * @brief   Relative error between last two approximations.
     * @details Termination occurs when the relative error between
     *          two consecutive iterates is at most xtol.
     */
    double xtol;
    /**
     * @brief   Orthogonality desired between fvec and its derivs.
     * @details Termination occurs when the cosine of the angle
                between fvec and any column of the Jacobian is at
                most gtol in absolute value.
     */
    double gtol;
    /**
     * @brief   Step used to calculate the Jacobian.
     * @details Should be slightly larger than the relative error in the
     *          user-supplied functions.
     */
    double epsilon;
    /**
     * @brief   Used in determining the initial step bound.
     * @details This bound is set to the product of stepbound and the
     *          Euclidean norm of diag*x if nonzero, or else to
     *          stepbound itself. In most cases stepbound should lie
     *          in the interval (0.1,100.0). Generally, the value
     *          100.0 is recommended.
     */
    double stepbound;
    /**
     * @brief   Used to set the maximum number of function evaluations.
     * @details Sets to patience*(number_of_parameters+1).
     */
    int patience;
    /**
     * @brief   If 1, the variables will be rescaled internally.
     * @details Recommended value is 1.
     */
    int scale_diag;
    /**
     * @brief   Progress messages will be written to this file.
     */
    FILE* msgfile;
    /**
     * @brief   Controls the ouput verbosity.
     * @details | Variations | Result              |
     *          |:-----------|:--------------------|
     *          | OR'ed: 1   | print some messages |
     *          | OR'ed: 2   | print Jacobian      |
     */
    int verbosity;
    /**
     * @brief   Maximum parameters to print
     * @details  -1, or max number.
     */
    int n_maxpri;
    /**
     * @brief   Max number of residuals to print
     * @details -1, or max number.
     */
    int m_maxpri;
} lm_control_struct;

/**
 * @brief   Collection of output parameters for status info.
 */
typedef struct {
    /**
     * @brief   norm of the residue vector fvec.
     */
    double fnorm;
    /**
     * @brief actual number of iterations.
     */
    int nfev;
    /**
     * @brief   Status indicator.
     * @details Nonnegative values are used as index for the message
     *          text lm_infmsg, set in lmmin.c.
     */
    int outcome;
    /**
     * @brief   Set when function evaluation requests termination.
     */
    int userbreak;
} lm_status_struct;

/**
 * @brief   Preset (and recommended) control parameter settings.
 * @details double-precision
 */
extern const lm_control_struct lm_control_double;
/**
 * @brief   Preset (and recommended) control parameter settings.
 * @details float-precision
 */
extern const lm_control_struct lm_control_float;

/**
 * @brief   Preset message texts.
 */
extern const char *lm_infmsg[];
/**
 * @brief   Preset message texts.
 */
extern const char *lm_shortmsg[];

__END_DECLS
#endif /* LMSTRUCT_H */
