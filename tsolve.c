#include <stdio.h>
#include <math.h>
#include <memory.h>
#include <unistd.h>
#include <sys/time.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_odeiv.h>
#include <gsl/gsl_multimin.h>

#define RTOD(x) ((x) / 2.0 / M_PI * 360)
#define DTOR(x) ((x) / 360.0 * 2 * M_PI)

int vflag;

FILE *outf;

struct experiment_params {
	double observed_hit[3];
	double observed_bounce[3];
	double observed_secs;

	double observed_dist;

	double alpha;

	double sim_dist;
	double sim_secs;

	gsl_vector *starting_point;

	int minimizer_dimen;
	gsl_vector *minimizer_step_sizes;

	int simulator_dimen;

	gsl_odeiv_step *stepper;
	gsl_odeiv_control *controller;
	gsl_odeiv_evolve *evolver;
	gsl_odeiv_system odesys;

	gsl_multimin_fminimizer *minimizer;
};

#define GRAVITY -9.81

int
sim_func (double t, const double state[], double dstate[], void *params_raw)
{
	struct experiment_params *params = params_raw;
	double v;

	if (0) {
		dstate[0] = state[2];
		dstate[1] = state[3];
		dstate[2] = 0;
		dstate[3] = -9.81;
	} else {
		v = hypot (state[2], state[3]);
		dstate[0] = state[2];
		dstate[1] = state[3];
		dstate[2] = - params->alpha * 0.508 * state[2] * v;
		dstate[3] = - params->alpha * 0.508 * state[3] * v + GRAVITY;
	}

	return GSL_SUCCESS;
}

void
do_simulation (const gsl_vector *proposed_vars, void *params_raw)
{
	struct experiment_params *params = params_raw;
	double ball_diameter, ball_mass, rho;
	double proposed_speed, proposed_angle;
	double state[params->simulator_dimen];
	double prev_state[params->simulator_dimen];
	double t, prev_t, frame_secs, step_size;
	int fine_mode;

	ball_diameter = 0.063;
	ball_mass = 0.05;
	rho = 1.29; 

	params->alpha = M_PI*(ball_diameter*ball_diameter)/(8*ball_mass)*rho;
#if 0	
	etha = 1; /* spin direction */
	w = 20; /* spin speed */
#endif

	proposed_speed = gsl_vector_get (proposed_vars, 0);
	proposed_angle = gsl_vector_get (proposed_vars, 1);

	state[0] = 0; // x
	state[1] = params->observed_hit[2]; // z
	state[2] = proposed_speed * cos(proposed_angle); // dx
	state[3] = proposed_speed * sin(proposed_angle); // dz

	gsl_odeiv_evolve_reset (params->evolver);

	t = 0;
	frame_secs = .030;
	step_size = 1e-6;
	fine_mode = 0;
	while (1) {
		memcpy (prev_state, state, sizeof state);
		prev_t = t;

		if (gsl_odeiv_evolve_apply (params->evolver,
					    params->controller,
					    params->stepper,
					    &params->odesys, 
					    &t, t + frame_secs,
					    &step_size,
					    state)
		    != GSL_SUCCESS) {
			fprintf (stderr, "evolve error\n");
			exit (1);
		}
		
		if (state[1] < 0) {
			fine_mode++;
			if (fine_mode > 4)
				break;
			memcpy (state, prev_state, sizeof state);
			t = prev_t;
			frame_secs /= 10;
		}

		if (vflag && fine_mode == 0)
			fprintf (outf, "%.14g %.14g\n", state[0], state[1]);
	}

	if (vflag) {
		fprintf (outf, "%.14g %.14g\n", prev_state[0], prev_state[1]);
	}

	params->sim_dist = prev_state[0];
	params->sim_secs = prev_t;
}

double
compute_error_func (const gsl_vector *proposed_vars, void *params_raw)
{
	struct experiment_params *params = params_raw;
	double dist_err, secs_err, err;
	char filename[1000];
	double proposed_speed, proposed_angle;

	proposed_speed = gsl_vector_get (proposed_vars, 0);
	proposed_angle = gsl_vector_get (proposed_vars, 1);

	if (vflag) {
		sprintf (filename, "sim%04.1f-%04.1f.dat",
			 proposed_speed, RTOD (proposed_angle));
		if ((outf = fopen (filename, "w")) == NULL) {
			fprintf (stderr, "can't create %s\n", filename);
			exit (1);
		}
		printf ("%s\n", filename);
	}

	do_simulation (proposed_vars, params);

	if (vflag) {
		fclose (outf);
	}

	dist_err = params->sim_dist - params->observed_dist;
	secs_err = params->sim_secs - params->observed_secs;

	err = dist_err * dist_err + 100 * secs_err * secs_err;

	if (vflag) {
		printf ("%10.6f %10.6f %10.6f %10.6f %10.6f %10.6f\n",
			params->sim_secs,
			gsl_vector_get (proposed_vars, 0),
			RTOD (gsl_vector_get (proposed_vars, 1)),
			dist_err, secs_err, err);
	}

	return (err);
}

void
solve_by_simulation (struct experiment_params *params)
{
       gsl_multimin_function func_info;
       int iter;
       double err;
     
       func_info.n = params->minimizer_dimen;
       func_info.f = compute_error_func;
       func_info.params = params;
     
       gsl_multimin_fminimizer_set (params->minimizer,
				    &func_info,
				    params->starting_point,
				    params->minimizer_step_sizes);
     
       iter = 0;
       while (1) {
	       iter++;
	       if (iter >= 200) {
		       fprintf (stderr, "can't find minimum\n");
		       exit (1);
	       }

	       if (gsl_multimin_fminimizer_iterate (params->minimizer)) {
		       fprintf (stderr, "iterate error\n");
		       exit (1);
	       }
     
	       err = gsl_multimin_fminimizer_minimum (params->minimizer);
	       if (fabs (err) < .001) {
		       if (vflag)
			       printf ("ok %d\n", iter);
		       break;
	       }
       }
}

void
graph_error_func (struct experiment_params *params)
{
	double speed, angle, err;
	FILE *f;
	gsl_vector *args;

	args = gsl_vector_alloc (params->minimizer_dimen);

	if ((f = fopen ("err.dat", "w")) == NULL) {
		fprintf (stderr, "can't create err.dat\n");
		exit (1);
	}

	for (speed = 0; speed < 80; speed += 1) {
		for (angle = DTOR (0); angle < DTOR (50); angle += DTOR (1)) {

			gsl_vector_set (args, 0, speed);
			gsl_vector_set (args, 1, angle);

			err = compute_error_func (args, params);
			fprintf (f, "%.14g %.14g %.14g\n",
				 speed, RTOD (angle), err);
		}
		fprintf (f, "\n");
	}
	fclose (f);
}
			
double
get_secs (void)
{
	struct timeval tv;
	gettimeofday (&tv, NULL);
	return (tv.tv_sec + tv.tv_usec / 1e6);
}

void
usage (void)
{
	fprintf (stderr, "usage: tsolve\n");
	exit (1);
}


#define METERS_PER_SEC_TO_MPH 2.2369363
int
main (int argc, char **argv)
{
	struct experiment_params params;
	double dx, dy;
	double speed, angle;
	gsl_vector *solution;
	int c;
	double compute_start, delta;

	while ((c = getopt (argc, argv, "v")) != EOF) {
		switch (c) {
		case 'v':
			vflag = 1;
			break;
		default:
			usage ();
		}
	}


	memset (&params, 0, sizeof params);
	params.observed_hit[0] = 0;
	params.observed_hit[1] = 0;
	params.observed_hit[2] = 1;
	params.observed_bounce[0] = 25;
	params.observed_bounce[1] = 0;
	params.observed_bounce[2] = 0;
	params.observed_secs = 1.359;

	dx = params.observed_bounce[0] - params.observed_hit[0];
	dy = params.observed_bounce[1] - params.observed_hit[1];
	params.observed_dist = hypot (dy, dx);

	params.simulator_dimen = 4;

	params.odesys.function = sim_func;
	params.odesys.dimension = params.simulator_dimen;
	params.odesys.params = &params;

	params.stepper = gsl_odeiv_step_alloc (gsl_odeiv_step_rk8pd,
					       params.simulator_dimen);
	params.controller = gsl_odeiv_control_y_new (1e-6, 0.0);
	params.evolver = gsl_odeiv_evolve_alloc (params.simulator_dimen);
	
	params.minimizer_dimen = 2;
	params.starting_point = gsl_vector_alloc (params.minimizer_dimen);

	params.minimizer_step_sizes = gsl_vector_alloc (params.minimizer_dimen);
	params.minimizer = gsl_multimin_fminimizer_alloc
		(gsl_multimin_fminimizer_nmsimplex2, params.minimizer_dimen);

       gsl_vector_set_all (params.starting_point, 0);
       gsl_vector_set_all (params.minimizer_step_sizes, 1.0);
     
	if (0) {
		graph_error_func (&params);
	}

	compute_start = get_secs ();
	solve_by_simulation (&params);
	delta = get_secs () - compute_start;

	solution = gsl_multimin_fminimizer_x (params.minimizer);

	speed = gsl_vector_get (solution, 0);
	angle = gsl_vector_get (solution, 1);

	printf ("speed = %8.3f angle = %8.3f; compute time %.3fms\n",
		speed * METERS_PER_SEC_TO_MPH,
		RTOD (angle),
		delta * 1000);

	return (0);
}
