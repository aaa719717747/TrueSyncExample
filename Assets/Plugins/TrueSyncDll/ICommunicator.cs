using System;

/// <summary>
/// TrueSync's communicator interface.
/// </summary>
public interface ICommunicator
{
    /// <summary>
    /// Returns the roundtrip time between local player and server. 
    /// </summary>
	int RoundTripTime();

    /// <summary>
    /// Raises a custom event to be sent to all other players.
    /// </summary>
    /// <param name="eventCode">Code of the custom event</param>
    /// <param name="message">Message to be sent in event's body</param>
    /// <param name="reliable">If true it should have a guaranteed delivery</param>
    /// <param name="toPlayers"></param>
	void OpRaiseEvent(byte eventCode, object message, bool reliable, int[] toPlayers);

    /// <summary>
    /// Adds an event listener to handle received custom events.
    /// </summary>
    /// <param name="onEventReceived">Implementation of OnEventReceived delegate.</param>
	void AddEventListener(OnEventReceived onEventReceived);
}
