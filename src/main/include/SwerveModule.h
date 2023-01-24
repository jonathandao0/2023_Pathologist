
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/MathUtil.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <ctre/phoenix/sensors/WPI_CANCoder.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/simulation/FlywheelSim.h>
#include <frc/system/plant/LinearSystemId.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>

#include "Constants.h"
#include "HardwareConfig.h"

class SwerveModule {
    public:
        SwerveModule(const double Module[]);
        void SetDesiredState(frc::SwerveModuleState& DesiredState, bool IsOpenLoop);
        frc::Rotation2d GetCANCoder();
        frc::SwerveModuleState Optimize(frc::SwerveModuleState DesiredState, frc::Rotation2d CurrentAngle);
        frc::SwerveModuleState GetState();
        void SetDegrees(units::degree_t Degrees);
        void SwapOrientation();   
        frc::SwerveModulePosition GetPosition();

        units::meter_t FalconToMeters(double Counts);    
        units::degree_t FalconToDegrees(double Counts);
        double DegreesToFalcon(units::degree_t Degrees);
        double FalconToRPM(double VelocityCounts);
        double RPMToFalcon(double RPM);
        double getTurnCounts();
        units::degree_t getLastAngle();
        units::meters_per_second_t FalconToMPS(double Velocitycounts);
        double MPSToFalcon(units::meters_per_second_t Velocity);
        

        void SimulationPeriodic();
    private:
        void resetToAbsolute();
        
        units::degree_t m_LastAngle;
        ctre::phoenix::motorcontrol::can::WPI_TalonFX m_DriveMotor;
        ctre::phoenix::motorcontrol::can::WPI_TalonFX m_AngleMotor;
        ctre::phoenix::sensors::WPI_CANCoder m_AngleEncoder;
        units::degree_t m_AngleOffset;

        frc::SimpleMotorFeedforward<units::feet> m_Feedforward;
        HardwareConfig m_Settings;


        frc::sim::FlywheelSim m_driveMotorSim{
            frc::LinearSystemId::IdentifyVelocitySystem<units::radian>(1.5_V / 1_rad_per_s, 0.6_V / 1_rad_per_s_sq),
            frc::DCMotor::Falcon500(1),
            SwerveConstants::DriveGearRatio
            };
        frc::sim::FlywheelSim m_angleMotorSim{
            frc::LinearSystemId::IdentifyVelocitySystem<units::radians>(0.1_V / 1_rad_per_s, 0.0001_V / 1_rad_per_s_sq),
            frc::DCMotor::Falcon500(1),
            SwerveConstants::AngleGearRatio
            };
        double m_drivePercentOutput = 0;
        double m_anglePercentOutput = 0;
        units::radian_t m_driveMotorSimDistance{0};
        units::radian_t m_angleMotorSimDistance{0};
};