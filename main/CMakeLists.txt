# Get git version at build time
execute_process(
    COMMAND git rev-parse --short HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_COMMIT_HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)

execute_process(
    COMMAND git rev-parse --abbrev-ref HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_BRANCH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)

execute_process(
    COMMAND git describe --tags --always
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)

execute_process(
    COMMAND git config --get remote.origin.url
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_REMOTE_URL
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)

# Check for uncommitted changes
execute_process(
    COMMAND git status --porcelain
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_STATUS_OUTPUT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)

# Check if ahead/behind remote
execute_process(
    COMMAND git rev-list --left-right --count origin/${GIT_BRANCH}...${GIT_BRANCH}
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_AHEAD_BEHIND
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)

# Add -dirty suffix if there are uncommitted changes or if local is ahead of remote
if(GIT_STATUS_OUTPUT OR GIT_AHEAD_BEHIND)
    if(NOT GIT_VERSION MATCHES "-dirty$")
        string(APPEND GIT_VERSION "-dirty")
    endif()
endif()

idf_component_register(
    SRCS "main.cpp" "Battery.cpp"
    INCLUDE_DIRS ""
    REQUIRES arduino-esp32-3.3.2 espressif__esp-zigbee-lib app_update nvs_flash
)

target_compile_definitions(${COMPONENT_LIB} PRIVATE
    GIT_COMMIT_HASH="${GIT_COMMIT_HASH}"
    GIT_BRANCH="${GIT_BRANCH}"
    GIT_VERSION="${GIT_VERSION}"
    GIT_REMOTE_URL="${GIT_REMOTE_URL}"
)
