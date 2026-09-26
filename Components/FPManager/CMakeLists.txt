register_fprime_library(
    AUTOCODER_INPUTS
        "${CMAKE_CURRENT_LIST_DIR}/FPManager.fpp"
    SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/FPManager.cpp"
    DEPENDS
      Types
      StateMachines_FPStateMachine
)

### Unit Tests ###
register_fprime_ut(
    AUTOCODER_INPUTS
        "${CMAKE_CURRENT_LIST_DIR}/FPManager.fpp"
    SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/test/ut/FPManagerTestMain.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/test/ut/FPManagerTester.cpp"
    DEPENDS
        Types
        StateMachines_FPStateMachine
    UT_AUTO_HELPERS
)
