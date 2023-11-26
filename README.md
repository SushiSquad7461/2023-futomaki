# futomatki-2023

## Suggestions

All of my suggestions will be in the form of proposed changes, PR comments, TODO comments, or here in this README

- Add a proper README, it doesn't have to be long

- Would recommend using a comprehensive style guide. I prefer [Google Java](https://google.github.io/styleguide/javaguide.html) (with some modifications), but whatever you use, you should setup a linter/autoformatter config. 

- I would consider enforcing the same units across all places in the code (e.g. meters, radians, seconds) and maybe either using proper units containers or naming variables to indicate their units.

- I'd also recommend enforcing max line length.

- I'd recommend switching from SmartDashboard to shuffleboard widgets so you can mandate their appearance from the code, or better yet use frc web components.

- The whole tuning mode is a good pattern at a base level, but I do feel a better solution is possible, not sure what that is yet

- If there's a single thing I'd recommend documenting almost *too* much it would be robot controls. Never hurts to have a quick, clear reference on that.