% Phased Array System Toolbox
% Version 3.0 (R2015a) 09-Feb-2015 
% 
% Table of Contents (T0C)
% -----------------------
%
% Array Analysis
%   pilotcalib                     - Array calibration using pilot sources
%   az2broadside                   - Convert azimuth to broadside angle
%   broadside2az                   - Convert broadside angle to azimuth
%   phased.ArrayGain               - Sensor array gain
%   phased.ArrayResponse           - Sensor array response
%   phased.ElementDelay            - Sensor array element delay estimator
%   phased.SteeringVector          - Sensor array steering vector
%   steervec                       - Sensor array steering vector
%
% Array Antenna Elements
%   aperture2gain                      - Convert effective aperture to gain
%   gain2aperture                      - Convert gain to effective aperture
%   phased.CosineAntennaElement        - Cosine antenna
%   phased.CrossedDipoleAntennaElement - Crossed dipole antenna
%   phased.CustomAntennaElement        - Custom antenna 
%   phased.IsotropicAntennaElement     - Isotropic antenna
%   phased.ShortDipoleAntennaElement   - Short dipole antenna
% 
% Array Design
%   phased.ConformalArray              - Conformal array
%   phased.HeterogeneousConformalArray - Heterogeneous conformal array
%   phased.HeterogeneousULA            - Heterogeneous uniform linear array
%   phased.HeterogeneousURA            - Heterogeneous uniform rectangular array
%   phased.PartitionedArray            - Phased array partitioned into subarrays
%   phased.ReplicatedSubarray          - Phased array formed by replicated subarrays
%   phased.UCA                         - Uniform circular array
%   phased.ULA                         - Uniform linear array
%   phased.URA                         - Uniform rectangular array
%
% Array Microphone Elements
%   phased.CustomMicrophoneElement          - Custom microphone
%   phased.OmnidirectionalMicrophoneElement - Omnidirectional microphone
%
% Beamformers
%   cbfweights                         - Narrowband conventional beamformer weights
%   lcmvweights                        - Narrowband LCMV beamformer weights
%   mvdrweights                        - Narrowband MVDR beamformer weights
%   phased.FrostBeamformer             - Frost beamformer
%   phased.LCMVBeamformer              - Narrowband LCMV beamformer
%   phased.MVDRBeamformer              - Narrowband MVDR beamformer
%   phased.PhaseShiftBeamformer        - Narrowband phase shift beamformer
%   phased.SubbandPhaseShiftBeamformer - Subband phase shift beamformer
%   phased.TimeDelayBeamformer         - Time delay beamformer
%   phased.TimeDelayLCMVBeamformer     - Time delay LCMV beamformer
%
% Clutter Models
%   billingsleyicm                      - Billingsley's intrinsic clutter motion (ICM) model  
%   depressionang                       - Depression angle of surface target
%   effearthradius                      - Effective earth radius
%   grazingang                          - Grazing angle of surface target
%   horizonrange                        - Horizon range
%   phased.ConstantGammaClutter         - Constant gamma clutter simulation
%   phased.gpu.ConstantGammaClutter     - Constant gamma clutter simulation using a GPU
%   surfacegamma                        - Gamma value for different terrains
%   surfclutterrcs                      - Surface clutter radar cross section
%
% Coordinate System and Motion Modeling
%   dop2speed                   - Convert Doppler shift to speed
%   global2localcoord           - Global to local coordinates conversion
%   local2globalcoord           - Local to global coordinates conversion
%   phased.Platform             - Motion platform
%   radialspeed                 - Relative radial speed
%   rangeangle                  - Range and angle calculation
%   speed2dop                   - Convert speed to Doppler shift
%
% Detection
%   albersheim                  - Albersheim's equation
%   npwgnthresh                 - Detection SNR threshold for white Gaussian noise
%   phased.CFARDetector         - Constant false alarm rate (CFAR) detector
%   phased.MatchedFilter        - Matched filter
%   phased.RangeDopplerResponse - Range-Doppler response
%   phased.StretchProcessor     - Stretch processor for linear FM waveform
%   phased.TimeVaryingGain      - Time varying gain control
%   pulsint                     - Pulse integration
%   rocpfa                      - Receiver operating characteristic curves on varying Pfa
%   rocsnr                      - Receiver operating characteristic curves on varying SNR
%   shnidman                    - Shnidman's equation
%   stretchfreq2rng             - Convert frequency offset from stretch processing to range
%
% Direction of Arrival (DOA)
%   espritdoa                       - ESPRIT direction of arrival (DOA)
%   phased.BeamscanEstimator        - Beamscan spatial spectrum estimator for ULA
%   phased.BeamscanEstimator2D      - 2-D Beamscan spatial spectrum estimator
%   phased.BeamspaceESPRITEstimator - Beamspace ESPRIT direction of arrival (DOA) estimator
%   phased.ESPRITEstimator          - ESPRIT direction of arrival (DOA) estimator
%   phased.MVDREstimator            - MVDR spatial spectrum estimator for ULA
%   phased.MVDREstimator2D          - 2-D MVDR spatial spectrum estimator
%   phased.RootMUSICEstimator       - Root MUSIC direction of arrival (DOA) estimator
%   phased.RootWSFEstimator         - Root WSF direction of arrival (DOA) estimator
%   phased.SumDifferenceMonopulseTracker   - Sum and difference monopulse for ULA
%   phased.SumDifferenceMonopulseTracker2D - Sum and difference monopulse for URA
%   rootmusicdoa                    - Root MUSIC direction of arrival (DOA)
%   spsmooth                        - Spatial smoothing of a covariance matrix
%
% Environment Models
%   fspl                        - Free space path loss
%   phased.FreeSpace            - Free space environment
%
% Jammer Models
%   phased.BarrageJammer        - Barrage jammer
%
% Polarization
%   circpol2pol                 - Circular to linear polarization representation conversion
%   pol2circpol                 - Linear to circular polarization representation conversion
%   polellip                    - Polarization ellipse
%   polloss                     - Polarization loss
%   polratio                    - Polarization ratio
%   polsignature                - Polarization signature 
%   stokes                      - Stokes parameters
%
% Radar Analysis
%   blakechart                  - Blake chart for radar
%   radareqpow                  - Radar equation to estimate power
%   radareqrng                  - Radar equation to estimate range
%   radareqsnr                  - Radar equation to estimate SNR
%   radarvcd                    - Vertical coverage diagram for radar
%
% Receiver Models
%   noisepow                    - Noise power at the receiver
%   phased.Collector            - Narrowband signal collector
%   phased.ReceiverPreamp       - Receiver preamp
%   phased.WidebandCollector    - Wideband signal collector
%   sensorcov                   - Covariance matrix of received signal
%   sensorsig                   - Received signal at sensor array
%   systemp                     - System noise temperature
%
% Space-Time Adaptive Processing (STAP)
%   dopsteeringvec              - Steering vector for Doppler
%   phased.ADPCACanceller       - Adaptive DPCA (ADPCA) pulse canceller
%   phased.AngleDopplerResponse - Angle-Doppler response
%   phased.DPCACanceller        - Displaced phase center array (DPCA) pulse canceller
%   phased.STAPSMIBeamformer    - Sample matrix inversion (SMI) STAP beamformer
%
% Target Models
%   phased.RadarTarget          - Radar target
%
% Transmitter Models
%   phased.Radiator             - Narrowband signal radiator
%   phased.Transmitter          - Transmitter
%
% Utilities
%   aictest          - Akaike information criterion test
%   azelaxes         - Axes at given azimuth and elevation direction
%   azel2phitheta    - Convert angles from az/el format to phi/theta format
%   azel2phithetapat - Convert pattern from az/el to phi/theta format
%   azel2uv          - Convert angles from az/el format to u/v format
%   azel2uvpat       - Convert pattern from az/el to u/v format
%   cart2sphvec      - Convert vector from Cartesian to Spherical representation
%   delayseq         - Delay or advance time sequence
%   mdltest          - Minimum description length test
%   physconst        - Physical constants of natural phenomena
%   phitheta2azel    - Convert angles from phi/theta format to az/el format
%   phitheta2azelpat - Convert pattern from phi/theta to az/el format
%   phitheta2uv      - Convert angles from phi/theta format to u/v format
%   phitheta2uvpat   - Convert pattern from phi/theta to u/v format
%   range2bw         - Convert range resolution to required bandwidth
%   range2time       - Convert propagation distance to propagation time
%   rotx             - Rotation matrix around x-axis
%   roty             - Rotation matrix around y-axis
%   rotz             - Rotation matrix around z-axis
%   sph2cartvec      - Convert vector from Spherical to Cartesian representation
%   time2range       - Convert propagation time to propagation distance
%   unigrid          - Generate uniform grid
%   uv2azel          - Convert angles from u/v format to az/el format
%   uv2azelpat       - Convert pattern from u/v to az/el format
%   uv2phitheta      - Convert angles from u/v format to phi/theta format
%   uv2phithetapat   - Convert pattern from u/v to phi/theta format
%   val2ind          - Convert the grid value to grid index
%
% Waveforms
%   ambgfun                     - Ambiguity function
%   beat2range                  - Convert beat frequency to range
%   dechirp                     - Dechirp FMCW signal
%   phased.FMCWWaveform         - FMCW waveform
%   phased.LinearFMWaveform     - Linear FM pulse waveform
%   phased.MFSKWaveform         - MFSK waveform
%   phased.PhaseCodedWaveform   - Phase-coded pulse waveform
%   phased.RectangularWaveform  - Rectangular pulse waveform
%   phased.SteppedFMWaveform    - Stepped FM pulse waveform
%   range2beat                  - Convert range to beat frequency
%   rdcoupling                  - Range Doppler coupling
% 
% Apps
%   radarWaveformAnalyzer       - Analyze performance characteristics of pulsed, 
%                                 frequency modulated, and phase coded waveforms
%   radarEquationCalculator     - Estimate maximum range, peak power and 
%                                 SNR of a radar system
%   sensorArrayAnalyzer         - Analyze beam pattern of linear, planar, 
%                                 and conformal sensor arrays
%
%   <a href="matlab:demo toolbox phased">Examples</a>                    - Phased Array System Toolbox examples
%   <a href="matlab:phasedlib">Simulink library</a>            - Open Phased Array System Toolbox Simulink library
%
% See also SIGNAL, DSP.

%   Copyright 2006-2015 The MathWorks, Inc.
%    

