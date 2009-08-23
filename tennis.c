#include <opus/opus.h>
#include <signal.h>

#include "SDL.h"
#include <GL/gl.h>
#include <GL/glu.h>

void reset_ball (void);

double
get_secs (void)
{
	struct timeval tv;
	static double start;
	double now;

	gettimeofday (&tv, NULL);
	now = tv.tv_sec + tv.tv_usec / 1e6;
	if (start == 0)
		start = now - .001;
	return (now - start);
}


#define TICK_INTERVAL 30

int width, height;

SDL_Surface *sdl_surface;

void
intr (int sig)
{
	SDL_Quit ();
	exit (0);
}

GLUquadric *ball_quad;

void
setup_opengl (void)
{
	double aspect_ratio;

	aspect_ratio = (double)width / height;

	glClearColor (0, 0, 0, 0);
	glViewport (0, 0, width, height);
	glMatrixMode (GL_PROJECTION);
	glLoadIdentity ();
	gluPerspective (130.0, aspect_ratio, 1.0, 1024.0);

	glShadeModel (GL_SMOOTH);
	glCullFace (GL_BACK);
	glFrontFace (GL_CCW);
	glEnable (GL_CULL_FACE);

	ball_quad = gluNewQuadric ();

}

/*
      +------------------+-------------------+ y2
      +--------+---------+---------+---------+ y1
      |        |         |         |         |
      |        |         |         |         |
      +        +---------0---------+         + 0
      |        |         |         |         |
      |        |         |         |         |
      +--------+---------+---------+---------+ -y1
      +------------------+-------------------+ -y2
     -x2      -x1        0        x1        x2

 */

#define X1 6.4
#define X2 11.89
#define Y1 4.115
#define Y2 5.485

double stripe_width;
double stripe_z;
void hstripe (double x1, double x2, double y);
void vstripe (double y1, double y2, double x);

double ball_pos[3];
double ball_vel[3];

double ball_r = .5;
double ball_mass = .2;

int net_flag;

void
freeze_ball (void)
{
	int i;
	for (i = 0; i < 3; i++)
		ball_vel[i] = 0;
}

void
physics (void)
{
	double t, dt;
	int i;
	static double last_t;
	double acc[3];

	t = get_secs ();
	dt = t - last_t;
	last_t = t;

	if (ball_pos[0] < -2 * X1)
		reset_ball ();

	acc[0] = 0;
	acc[1] = 0;
	acc[2] = -9.8;

	for (i = 0; i < 3; i++)
		ball_vel[i] += dt * acc[i];

	for (i = 0; i < 3; i++)
		ball_pos[i] += dt * ball_vel[i];

	if (net_flag == 0 && ball_pos[0] < 0) {
		net_flag = 1;
		printf ("crossed net at %.3f m\n", ball_pos[2]);
	}

	if (ball_pos[2] < 0) {
		printf ("bounce at %.3f %.3f\n", ball_pos[0], ball_pos[1]);

		ball_pos[2] = 0;
		ball_vel[2] *= -.9;

		if (0) {
			freeze_ball ();
		}
	}

}

void
draw (void)
{
	physics ();

	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	
	glMatrixMode( GL_MODELVIEW );

	glLoadIdentity ();
	gluLookAt (0, -5, 4, /* eye */
		   0, 4, 0, /* center */
		   0, 0, 1); /* up */

	glBegin (GL_LINES);
	glColor3f (1, 0, 0);
	glVertex3f (0, 0, 0);
	glVertex3f (1000, 0, 0);
	glColor3f (0, 1, 0);
	glVertex3f (0, 0, 0);
	glVertex3f (0, 1000, 0);
	glColor3f (0, 0, 1);
	glVertex3f (0, 0, 0);
	glVertex3f (0, 0, 1000);

	glEnd ();

	stripe_width = .2;
	stripe_z = .2;

	hstripe (-X2, X2, -Y2);
	hstripe (-X2, X2, -Y1);
	hstripe (-X2, X2, Y1);
	hstripe (-X2, X2, Y2);
	hstripe (-X1, X1, 0);

	vstripe (-Y2, Y2, -X2);
	vstripe (-Y1, Y1, -X1);
	vstripe (-Y2, Y2, 0);
	vstripe (-Y1, Y1, X1);
	vstripe (-Y2, Y2, X2);
	
	glPushMatrix ();
	glTranslated (ball_pos[0], ball_pos[1], ball_pos[2]);
	glColor3f (1, 1, 0);
	gluSphere (ball_quad, ball_r, 6, 6);
	glPopMatrix ();
}

void
hstripe (double x1, double x2, double y)
{
	glBegin (GL_QUADS);
	glColor3d (1, 1, 1);
	glVertex3d (x1, y - stripe_width, stripe_z);
	glVertex3d (x2, y - stripe_width, stripe_z);
	glVertex3d (x2, y + stripe_width, stripe_z);
	glVertex3d (x1, y + stripe_width, stripe_z);
	glEnd ();
}

void
vstripe (double y1, double y2, double x)
{
	glBegin (GL_QUADS);
	glColor3d (1, 1, 1);
	glVertex3d (x - stripe_width, y1, stripe_z);
	glVertex3d (x + stripe_width, y1, stripe_z);
	glVertex3d (x + stripe_width, y2, stripe_z);
	glVertex3d (x - stripe_width, y2, stripe_z);
	glEnd ();
}


void
process_events (void)
{
	SDL_Event ev;

	while (SDL_PollEvent (&ev)) {
		switch (ev.type) {
		case SDL_KEYDOWN:
			switch (ev.key.keysym.sym) {
			case SDLK_ESCAPE:
			case 'q':
				exit (0);

			default:
				break;
			}
			break;
		case SDL_QUIT:
			exit (0);
		}
	}
}

double
vmag (double *arr)
{
	return (sqrt (arr[0] * arr[0]
		      + arr[1] * arr[1]
		      + arr[2] * arr[2]));
}

void
reset_ball (void)
{
	ball_pos[0] = X2;
	ball_pos[1] = Y1;
	ball_pos[2] = 2.5;

	ball_vel[0] = -18;
	ball_vel[1] = -7;
	ball_vel[2] = 2;

	printf ("sim speed %.3f MPH\n", vmag (ball_vel) * 2.2369363);

	net_flag = 0;
}

int
main (int argc, char **argv)
{
	Uint32 next, now;
	int delta;

	if (SDL_Init (SDL_INIT_VIDEO) == -1)
		fatal ("can't init sdl");
	atexit (SDL_Quit);
	signal (SIGINT, intr);

	width = 320;
	height = 240;

	SDL_GL_SetAttribute (SDL_GL_DOUBLEBUFFER, 1);

	if ((sdl_surface = SDL_SetVideoMode (width, height, 32,
					     SDL_OPENGL)) == NULL)
		fatal ("SDL_SetVideoMode");

	setup_opengl ();

	reset_ball ();

	next = SDL_GetTicks ();
	while (1) {
		process_events ();

		draw ();

		SDL_GL_SwapBuffers ();

		next += TICK_INTERVAL;
		now = SDL_GetTicks ();
		delta = next - now;

		if (delta > 0)
			SDL_Delay (delta);
	}

	return (0);
}
