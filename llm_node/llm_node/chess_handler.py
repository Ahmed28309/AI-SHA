"""
Chess handler — self-contained chess capability for the LLM node.

Detects chess intent from speech, manages a game with Stockfish,
and returns text responses for TTS. No extra nodes or topics needed.
"""

import chess
import chess.engine
import re
import os

# ---------------------------------------------------------------------------
# Intent detection
# ---------------------------------------------------------------------------

_START_PHRASES = [
    "let's play chess", "lets play chess", "let us play chess",
    "play chess", "start chess", "chess game", "new chess game",
    "want to play chess", "play a game of chess", "game of chess",
]

_QUIT_PHRASES = [
    "quit chess", "stop chess", "end chess", "exit chess",
    "i resign", "i give up", "resign", "stop the game",
    "end the game", "quit the game", "stop playing",
]

# ---------------------------------------------------------------------------
# Spoken-move mappings
# ---------------------------------------------------------------------------

_PIECE_MAP = {
    'pawn': None, 'knight': chess.KNIGHT, 'night': chess.KNIGHT,
    'horse': chess.KNIGHT, 'bishop': chess.BISHOP, 'rook': chess.ROOK,
    'castle': chess.ROOK, 'tower': chess.ROOK, 'queen': chess.QUEEN,
    'king': chess.KING,
}

_PIECE_SPOKEN = {
    chess.PAWN: 'pawn', chess.KNIGHT: 'knight', chess.BISHOP: 'bishop',
    chess.ROOK: 'rook', chess.QUEEN: 'queen', chess.KING: 'king',
}

_FILE_MAP = {
    'a': 'a', 'alpha': 'a', 'ay': 'a',
    'b': 'b', 'bravo': 'b', 'bee': 'b',
    'c': 'c', 'charlie': 'c', 'see': 'c', 'sea': 'c',
    'd': 'd', 'delta': 'd', 'dee': 'd',
    'e': 'e', 'echo': 'e',
    'f': 'f', 'foxtrot': 'f', 'ef': 'f',
    'g': 'g', 'golf': 'g', 'gee': 'g',
    'h': 'h', 'hotel': 'h',
}

_RANK_MAP = {
    '1': '1', 'one': '1', 'won': '1',
    '2': '2', 'two': '2', 'to': '2', 'too': '2',
    '3': '3', 'three': '3',
    '4': '4', 'four': '4', 'for': '4',
    '5': '5', 'five': '5',
    '6': '6', 'six': '6',
    '7': '7', 'seven': '7',
    '8': '8', 'eight': '8', 'ate': '8',
}

_SKIP_WORDS = {'to', 'takes', 'take', 'captures', 'capture', 'on', 'at'}


# ---------------------------------------------------------------------------
# ChessHandler
# ---------------------------------------------------------------------------

class ChessHandler:
    """Drop-in chess capability. Call handle(text) and get a response string back."""

    STOCKFISH_PATH = '/usr/games/stockfish'

    def __init__(self, skill_level=5, think_time=0.5):
        self.board = chess.Board()
        self.active = False
        self.engine = None
        self.skill_level = skill_level
        self.think_time = think_time

    # ------------------------------------------------------------------
    # Public API — called from LLM node
    # ------------------------------------------------------------------

    def is_chess_intent(self, text):
        """Return True if text looks like it wants to start/play/quit chess."""
        t = text.lower()
        if self.active:
            return True  # everything goes through chess while a game is on
        return any(p in t for p in _START_PHRASES)

    def handle(self, text):
        """
        Process text and return a response string (for TTS).
        Returns None if the text isn't chess-related.
        """
        t = text.lower().strip()

        # --- start a new game ---
        if not self.active and any(p in t for p in _START_PHRASES):
            return self._start_game()

        if not self.active:
            return None

        # --- quit / resign ---
        if any(p in t for p in _QUIT_PHRASES):
            return self._end_game("You resign. Good game! I'm ready to chat again.")

        # --- parse player move ---
        move, err = self._parse_move(t)
        if move is None:
            return err

        # apply player move
        player_announce = self._announce(move, perspective='player')
        self.board.push(move)

        # check game state after player move
        over_msg = self._check_game_over()
        if over_msg:
            return f"{player_announce} {over_msg}"

        # --- robot replies with Stockfish ---
        robot_response = self._robot_move()
        return f"{player_announce} {robot_response}"

    @property
    def is_active(self):
        return self.active

    # ------------------------------------------------------------------
    # Engine management
    # ------------------------------------------------------------------

    def _ensure_engine(self):
        if self.engine is not None:
            return True
        if not os.path.exists(self.STOCKFISH_PATH):
            return False
        try:
            self.engine = chess.engine.SimpleEngine.popen_uci(self.STOCKFISH_PATH)
            self.engine.configure({
                'Skill Level': self.skill_level,
                'Threads': 2,
                'Hash': 32,
            })
            return True
        except Exception:
            self.engine = None
            return False

    def _shutdown_engine(self):
        if self.engine:
            try:
                self.engine.quit()
            except Exception:
                pass
            self.engine = None

    # ------------------------------------------------------------------
    # Game lifecycle
    # ------------------------------------------------------------------

    def _start_game(self):
        if not self._ensure_engine():
            return ("I'd love to play chess but Stockfish isn't installed. "
                    "Ask someone to run sudo apt install stockfish.")
        self.board = chess.Board()
        self.active = True
        return ("Game on! You're white, I'm black. "
                "Tell me your move, like pawn to e4 or knight to f3.")

    def _end_game(self, msg):
        self.active = False
        return msg

    def _check_game_over(self):
        if self.board.is_checkmate():
            side = "You win, checkmate!" if self.board.turn == chess.BLACK else "Checkmate, I win!"
            self.active = False
            return side + " Good game. I'm ready to chat again."
        if self.board.is_stalemate():
            self.active = False
            return "Stalemate, it's a draw. Good game!"
        if self.board.is_insufficient_material():
            self.active = False
            return "Draw, not enough pieces left. Good game!"
        if self.board.is_check():
            return "Check!"
        return None

    # ------------------------------------------------------------------
    # Robot move via Stockfish
    # ------------------------------------------------------------------

    def _robot_move(self):
        if not self._ensure_engine():
            self.active = False
            return "I lost my chess engine mid-game. Let's try again later."
        try:
            result = self.engine.play(self.board, chess.engine.Limit(time=self.think_time))
            move = result.move
        except Exception:
            self.active = False
            return "Something went wrong with my thinking. Game over, sorry!"

        announce = self._announce(move, perspective='robot')
        self.board.push(move)

        over_msg = self._check_game_over()
        if over_msg:
            return f"{announce} {over_msg}"
        return f"{announce} Your turn."

    # ------------------------------------------------------------------
    # Move parsing (speech → chess.Move)
    # ------------------------------------------------------------------

    def _parse_move(self, text):
        """Return (move, error_string). One of them is always None."""
        # castling
        cm = self._try_castling(text)
        if cm:
            return cm, None

        words = text.split()
        piece_type = None
        rest = []
        for w in words:
            if w in _PIECE_MAP and piece_type is None:
                piece_type = _PIECE_MAP[w]
            elif w in _SKIP_WORDS:
                continue
            else:
                rest.append(w)

        sq = self._extract_square(rest)
        if sq is not None:
            target = chess.parse_square(sq)
            candidates = [
                m for m in self.board.legal_moves
                if m.to_square == target and (
                    piece_type is None or
                    self.board.piece_at(m.from_square).piece_type == piece_type
                )
            ]
            # if no piece specified and ambiguous, prefer pawns
            if piece_type is None and len(candidates) > 1:
                pawns = [m for m in candidates
                         if self.board.piece_at(m.from_square).piece_type == chess.PAWN]
                if len(pawns) == 1:
                    candidates = pawns

            if len(candidates) == 1:
                move = candidates[0]
                # auto-promote to queen
                if self._is_promotion(move):
                    move = chess.Move(move.from_square, move.to_square, promotion=chess.QUEEN)
                return move, None
            if len(candidates) > 1:
                options = ', '.join(self.board.san(m) for m in candidates)
                return None, f"That's ambiguous. Did you mean {options}?"
            return None, f"That's not a legal move. Try again."

        # fallback: try raw SAN / UCI
        return self._try_direct(text)

    def _try_castling(self, text):
        qs = ['castle queenside', 'castle queen side', 'long castle', 'queenside castle', 'o-o-o']
        ks = ['castle kingside', 'castle king side', 'short castle', 'kingside castle', 'o-o']
        for p in qs:
            if p in text:
                for m in self.board.legal_moves:
                    if self.board.is_castling(m) and self.board.is_queenside_castling(m):
                        return m
        for p in ks:
            if p in text:
                for m in self.board.legal_moves:
                    if self.board.is_castling(m) and not self.board.is_queenside_castling(m):
                        return m
        return None

    def _extract_square(self, words):
        for i in range(len(words)):
            f = _FILE_MAP.get(words[i])
            if f and i + 1 < len(words):
                r = _RANK_MAP.get(words[i + 1])
                if r:
                    return f + r
        for w in words:
            if re.match(r'^[a-h][1-8]$', w):
                return w
        return None

    def _try_direct(self, text):
        clean = text.strip()
        for prefix in ('play ', 'move ', 'i play ', 'my move is '):
            if clean.startswith(prefix):
                clean = clean[len(prefix):]
        try:
            return self.board.parse_san(clean), None
        except ValueError:
            pass
        try:
            m = chess.Move.from_uci(clean)
            if m in self.board.legal_moves:
                return m, None
        except ValueError:
            pass
        return None, "I didn't understand that move. Try something like pawn to e4."

    def _is_promotion(self, move):
        piece = self.board.piece_at(move.from_square)
        if piece and piece.piece_type == chess.PAWN:
            rank = chess.square_rank(move.to_square)
            if (piece.color == chess.WHITE and rank == 7) or \
               (piece.color == chess.BLACK and rank == 0):
                return True
        return False

    # ------------------------------------------------------------------
    # Move announcement
    # ------------------------------------------------------------------

    def _square_name(self, sq):
        f = 'abcdefgh'[chess.square_file(sq)]
        r = str(chess.square_rank(sq) + 1)
        return f'{f} {r}'

    def _announce(self, move, perspective='robot'):
        board = self.board  # state before push
        if board.is_castling(move):
            side = 'queenside' if board.is_queenside_castling(move) else 'kingside'
            if perspective == 'robot':
                return f"I castle {side}."
            return f"You castle {side}."

        piece = board.piece_at(move.from_square)
        pname = _PIECE_SPOKEN.get(piece.piece_type, 'piece')
        target = self._square_name(move.to_square)
        captured = board.piece_at(move.to_square)
        if board.is_en_passant(move):
            captured = chess.Piece(chess.PAWN, not piece.color)

        if perspective == 'robot':
            if captured:
                cname = _PIECE_SPOKEN.get(captured.piece_type, 'piece')
                return f"I take your {cname} on {target} with my {pname}."
            return f"I play {pname} to {target}."
        else:
            if captured:
                cname = _PIECE_SPOKEN.get(captured.piece_type, 'piece')
                return f"You take {cname} on {target} with {pname}."
            return f"You play {pname} to {target}."
