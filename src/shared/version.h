/* Protocol version shared between firmware and driver.
 * PROTOCOL_VERSION_PATCH is auto-incremented by the pre-commit hook.
 * PROTOCOL_VERSION_BRANCH is 0 on main; FNV-1a hash of branch name otherwise.
 * Both are updated by the pre-commit hook. */
#ifndef VERSION_H
#define VERSION_H
#define PROTOCOL_VERSION_MAJOR   0
#define PROTOCOL_VERSION_MINOR   2
#define PROTOCOL_VERSION_PATCH   26
#define PROTOCOL_VERSION_BRANCH  1567223694
#endif  // VERSION_H
