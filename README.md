# futomatki-2023

## Suggestions

All of my suggestions will be in the form of proposed changes, PR comments, TODO comments, or here in this README

- Add a proper README, it doesn't have to be long

- Would recommend using a comprehensive style guide. I prefer [Google Java](https://google.github.io/styleguide/javaguide.html) (with some modifications), but whatever you use, you should setup a linter/autoformatter config. 

- The WPILib naming conventions you follow are from the olden days of C and such when data types were not as well defined (AKA everything is stored as an int or something and interpreted where needed). It's not necessary, but I would recommend moving from the whole `kWhatever` naming scheme for constant classes to something else, as I feel it's just less intuitive for the reader despite its terseness.

- I would consider enforcing the same units across all places in the code (e.g. meters, radians, seconds) and maybe either using proper units containers or naming variables to indicate their units.

- I'd also recommend enforcing max line length.

- I would also like to know what the motivation is for using singleton classes for the subsystems outside of convenience.

- Try to minimize the amount of patterns for command construction. I see you use at least a mix of factory pattern, functional composition, and standard construction. particularly I'd recommend maximizing usage of functional composition, and using a factory for anything else. 

- I'd recommend switching from SmartDashboard to shuffleboard widgets so you can mandate their appearance from the code, or better yet use frc web components.

- AdvantageKit

- Of course the issues we discussed with the coordinate system non-sense. Basically just stick to the WPILib (and Limelight and PhotonVision and PathPlanner) conventions

- The whole tuning mode is a good pattern at a base level, but I do feel a better solution is possible, not sure what that is yet

- If there's a single thing I'd recommend documenting almost *too* much it would be robot controls. Never hurts to have a quick, clear reference on that.