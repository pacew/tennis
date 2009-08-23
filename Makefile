CFLAGS = -g -Wall `pkg-config --cflags sdl gl gsl`
LIBS = `pkg-config --libs sdl gl gsl` -lGLU -lopus -lm

tsolve: tsolve.o
	$(CC) $(CFLAGS) -o tsolve tsolve.o $(LIBS)

odetest: odetest.o
	$(CC) $(CFLAGS) -o odetest odetest.o $(LIBS)

tennis: tennis.o
	$(CC) $(CFLAGS) -o tennis tennis.o $(LIBS)
