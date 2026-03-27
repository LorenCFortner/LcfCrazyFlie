"""Tests for FlowDeck.

The FlowDeck class is currently a stub (implementation pending).
These tests verify the class exists and is importable.
"""

from Crazyflie.decks.flow_deck import FlowDeck


def test_flow_deck_class_is_importable():
    assert FlowDeck is not None


def test_flow_deck_can_be_instantiated():
    deck = FlowDeck()
    assert isinstance(deck, FlowDeck)
