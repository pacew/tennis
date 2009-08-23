#include <stdio.h>
#include <math.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_odeiv.h>
     
#define DIMEN 4

double g;
double alpha, w, etha;

FILE *outf;

int
tennis_vacuum (double t, const double y[], double ydot[], void *params)
{
	ydot[0] = y[2];
	ydot[1] = y[3];
	ydot[2] = 0;
	ydot[3] = g;

	return GSL_SUCCESS;
}
     
int
tennis_air (double t, const double y[], double ydot[], void *params)
{
	double v;

	v = hypot (y[2], y[3]);

	ydot[0] = y[2];
	ydot[1] = y[3];
	ydot[2] = -alpha*0.508*y[2]*v;
	ydot[3] = g-alpha*0.508*y[3]*v;

	return GSL_SUCCESS;
}
     
int
tennis_spin (double t, const double y[], double ydot[], void *params)
{
	double v, Cd, Cm;

	v = hypot (y[2], y[3]);

	Cd=(0.508+1/(22.503+4.196*pow (v/w, 0.4)))*alpha*v;
	Cm=etha*w/(2.022*w+0.981*v)*alpha*v;
	ydot[0] = y[2];
	ydot[1] = y[3];
	ydot[2] = -Cd*y[2]+Cm*y[3];
	ydot[3] = g-Cd*y[3]-Cm*y[2];

	return GSL_SUCCESS;
}
     
gsl_odeiv_step *stepper;
gsl_odeiv_control *controller;
gsl_odeiv_evolve *evolver;
gsl_odeiv_system odesys;
double state[DIMEN];

void
setup (int (*func)(double t, const double y[], double ydot[], void *params))
{
	double d, m, rho, v0;
	double height;
	double theta;

	odesys.function = func;
	odesys.jacobian = NULL;
	odesys.dimension = DIMEN;

	stepper = gsl_odeiv_step_alloc (gsl_odeiv_step_rk8pd, odesys.dimension);
	controller  = gsl_odeiv_control_y_new (1e-6, 0.0);
	evolver = gsl_odeiv_evolve_alloc (odesys.dimension);
	
	g = -9.81;
	d = 0.063;
	m = 0.05;
	rho = 1.29; 
	alpha = M_PI*(d*d)/(8*m)*rho;
	etha = 1;  
	w = 20;

	// initial condition
	height = 1;
	v0 = 25;
	theta=M_PI/180*15;
	state[0] = 0;
	state[1] = height;
	state[2] = v0*cos(theta);
	state[3] = v0*sin(theta);

}
	
void
run (char *filename)
{
	double t, t1;
	double step_size;

	if ((outf = fopen (filename, "w")) == NULL) {
		fprintf (stderr, "can't create %s\n", filename);
		exit (1);
	}


	step_size = 1e-6;
	t = 0.0;
	t1 = 5.0;

	while (1) {
		if (t >= t1)
			break;

		if (gsl_odeiv_evolve_apply (evolver,
					    controller,
					    stepper,
					    &odesys, 
					    &t, t + .005,
					    &step_size, state)
		    != GSL_SUCCESS) {
			break;
		}
		
		if (state[1] < 0)
			break;

		printf ("%8.3f %8.3f %8.3f %10.6f\n",
			t, state[0], state[1], step_size);

		fprintf (outf, "%.14g %.14g\n", state[0], state[1]);
	}

	fclose (outf);
}
	
void
finish () 
{
	gsl_odeiv_evolve_free (evolver);
	gsl_odeiv_control_free (controller);
	gsl_odeiv_step_free (stepper);
}

int
main (int argc, char **argv)
{
	setup (tennis_vacuum);
	run ("vacuum.dat");
	finish ();

	if (0) {
		setup (tennis_air);
		run ("air.dat");
		finish ();

		setup (tennis_spin);
		run ("spin.dat");
		finish ();
	}

	return (0);
}
