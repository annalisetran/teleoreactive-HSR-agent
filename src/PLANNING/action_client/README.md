# action_clients

## subscription topic:
- '/unsw_actions'
    - UNSWActionMsg
        - String action_name
        - String data -> JSON format

## publishing topics
- /unsw_actions/results
    - UNSWActionResult
        - String action_name
        - bool result

# Actions
## NAVIGATION
- goto_goal

## MANIPULATION
- approach_goal
- grasp_goal
- place_goal
- retract
