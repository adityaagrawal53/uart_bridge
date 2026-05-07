#!/usr/bin/env bash

SESSION="myproject"

# kill old session if it exists
tmux kill-session -t $SESSION 2>/dev/null

# CREATE SESSION + FIRST WINDOW
tmux new-session -d -s $SESSION -n editor

# pane 0 (default pane)
tmux send-keys -t $SESSION:editor.0 \
  "cd ~/projects/myproject && nvim" C-m

# CREATE SECOND WINDOW
tmux new-window -t $SESSION -n backend

# pane 0
tmux send-keys -t $SESSION:backend.0 \
  "cd ~/projects/myproject/backend && npm run dev" C-m

# split vertically (left/right)
tmux split-window -h -t $SESSION:backend

# pane 1
tmux send-keys -t $SESSION:backend.1 \
  "cd ~/projects/myproject/backend && tail -f logs/app.log" C-m

# split horizontally (top/bottom)
tmux split-window -v -t $SESSION:backend.1

# pane 2
tmux send-keys -t $SESSION:backend.2 \
  "htop" C-m

# apply layout
tmux select-layout -t $SESSION:backend tiled

# =========================================================
# CREATE THIRD WINDOW
# =========================================================

tmux new-window -t $SESSION -n frontend

# pane 0
tmux send-keys -t $SESSION:frontend.0 \
  "cd ~/projects/myproject/frontend && npm run dev" C-m

# split left/right
tmux split-window -h -t $SESSION:frontend

# pane 1
tmux send-keys -t $SESSION:frontend.1 \
  "cd ~/projects/myproject/frontend && lazygit" C-m

# =========================================================
# OPTIONAL WINDOW
# =========================================================

tmux new-window -t $SESSION -n shell

tmux send-keys -t $SESSION:shell \
  "cd ~/projects/myproject" C-m

# =========================================================
# STARTUP SETTINGS
# =========================================================

# choose initial window
tmux select-window -t $SESSION:editor

# attach
tmux attach-session -t $SESSION
