#ifndef _myAnalyzer_h
#define _myAnalyzer_h
typedef enum {STEP, FREQ, VECT, RAMP, RAND, SQUARE} testType;
static const double pi = 3.14159265358979323846264338327950288419716939937510;

class FRAnalyzer
{
public:
  FRAnalyzer();
  FRAnalyzer(const double omegaLogMin, const double omegaLogMax, const double deltaOmegaLog, const int minCycles,
             const double numCycleScalar, const int numInitCycles, const double wSlow, const double T, const int ix[], const int iy[],
             const int nsig, const int ntf, const String inHeader);
  ~FRAnalyzer(){};
  // operators
  // functions
  double calculate(const double *sig, const int nsig);
  bool complete(void) { return (complete_); };
  double omega(void) { return (omega_); };
  void complete(const bool set);
  void publish(void);

private:
  double calculateINI_(void);
  double calculateRUN_(void);
  double calculateSET_(void);
  void initializeINI_(void);
  void initializeRUN_(void);
  void initializeSET_(void);
  double properOmega_(const double updateTime, const int numCycles, const double omegaLog, unsigned long *iTargetOmega);
  void runIntegrate_(void);
  enum Mode
  {
    WAI,          // Wait
    SET,          // Set
    INI,          // Initialize
    RUN,          // Run
    CPT           // Complete
  } frMode_;                    // Run mode
  double *a1_;                  // Fourier series coefficient
  double aint_;                 // Mantissa buffer for modf call
  double *b1_;                  // Fourier series coefficient
  double cosOmT_;               // Memory of intermediate calculation
  bool complete_;               // If done, T/F
  double deltaOmegaLog_;        // Delta log10(frequency, r/s) in sweep
  double excite_;               // Excitation, fraction [-0.5, 0.5]
  String inHeader_;             // csv header prior to fr data
  unsigned long iOmega_;        // Current time step number at frequency
  unsigned int iResults_;       // Current number of results
  unsigned long iTargetOmega_;  // Time step target at frequency
  unsigned int iTargetResults_; // Target result limit
  unsigned int *ix_;            // Indeces of inputs to TFs
  unsigned int *iy_;            // Indeces of outputs to TFs, corresponding ot ix_
  unsigned int nsig_;           // Number of signals derived [from] sig[] size
  unsigned int ntf_;            // Number of transfer functions derived from ix[] size
  unsigned int minCycles_;      // Minimum number of sweep cycles at each frequency
  unsigned int numCycles_;      // Number of cycles to run of sinusoid
  double numCycleScalar_;       // Scalar times number of cycles normally done
  double omega_;                // Excitation frequency, r/s
  double omegaLogMax_;          // Maximum log10(frequency, r/s), of sweep
  double omegaLogMin_;          // Minimum log10(frequency, r/s), of sweep
  double omegaLog_;             // Excitation frequency log10
  double *sig_;                 // Signal pointers from sig input
  double *sigGain_;             // Signal gains, dB
  double *sigPhas_;             // Signal phase, deg
  double sinOmT_;               // Memory of intermediate calculation
  double T_;                    // Update time, sec
  double tau_;                  // Time constant slowest expected mode, sec
  double Tlog_;                 // Log10 of update time
  double timeTotalSweep_;       // Total expected time to complete sweep, sec
  double timeTargetOmega_;      // Target time at frequency, sec
  double timeAtOmega_;          // Time at frequency point, sec
  double *transGain_;           // Transfer function gains, dB
  double *transPhas_;           // Transfer function phase, deg
  double wSlow_;                // Frequency of slowest expected mode, r/s
};

class myVector
{
public:
  myVector();
  myVector(const double tv[], const double vv[], const int nv);
  ~myVector(){};
  // operators
  // functions
  double calculate(const double tnow);
  bool complete(void) { return (complete_); };
  void complete(const bool set);
private:
  bool complete_;               // If done, T/F
  unsigned int iv_;             // Location in vector
  unsigned int nv_;             // Length of vector
  double output_;               // State of output
  double time_;                 // Run time, sec
  double tnowStart_;            // First calculate call
  double *tv_;                  // Time vector, sec
  double *vv_;                  // Output vector, sec
};

#endif
