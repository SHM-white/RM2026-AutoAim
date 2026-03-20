add_library(sentry_decision sentry_decision.cpp)

target_include_directories(sentry_decision PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(sentry_decision PUBLIC io)
