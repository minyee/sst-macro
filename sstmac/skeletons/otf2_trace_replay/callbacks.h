/*
 *  This file is part of SST/macroscale:
 *               The macroscale architecture simulator from the SST suite.
 *  Copyright (c) 2009 Sandia Corporation.
 *  This software is distributed under the BSD License.
 *  Under the terms of Contract DE-AC04-94AL85000 with Sandia Corporation,
 *  the U.S. Government retains certain rights in this software.
 *  For more information, see the LICENSE file in the top
 *  SST/macroscale directory.
 */

#ifndef CALLBACKS_H_
#define CALLBACKS_H_

#include <otf2/otf2.h>

/******************************************************************************
 * Definition Callbacks
 *
 * OTF2 definition reader will use these as callbacks when streaming through a
 * definition file. Each callback should append a hash table. The event reader
 * callbacks will have parameters that reference these tables
 */

OTF2_CallbackCode def_clock_properties(
    void*    userData,
    uint64_t timerResolution,
    uint64_t globalOffset,
    uint64_t traceLength);

OTF2_CallbackCode def_string(
    void*          userData,
    OTF2_StringRef self,
    const char*    string );

OTF2_CallbackCode def_location(
    void*                 userData,
    OTF2_LocationRef      self,
    OTF2_StringRef        name,
    OTF2_LocationType     locationType,
    uint64_t              numberOfEvents,
    OTF2_LocationGroupRef locationGroup );

OTF2_CallbackCode def_location_group(
    void*                  userData,
    OTF2_LocationGroupRef  self,
    OTF2_StringRef         name,
    OTF2_LocationGroupType locationGroupType,
    OTF2_SystemTreeNodeRef systemTreeParent );

OTF2_CallbackCode def_region(
    void*           userData,
    OTF2_RegionRef  self,
    OTF2_StringRef  name,
    OTF2_StringRef  canonicalName,
    OTF2_StringRef  description,
    OTF2_RegionRole regionRole,
    OTF2_Paradigm   paradigm,
    OTF2_RegionFlag regionFlags,
    OTF2_StringRef  sourceFile,
    uint32_t        beginLineNumber,
    uint32_t        endLineNumber );

OTF2_CallbackCode def_callpath(
    void*            userData,
    OTF2_CallpathRef self,
    OTF2_CallpathRef parent,
    OTF2_RegionRef   region );

OTF2_CallbackCode def_group(
    void*           userData,
    OTF2_GroupRef   self,
    OTF2_StringRef  name,
    OTF2_GroupType  groupType,
    OTF2_Paradigm   paradigm,
    OTF2_GroupFlag  groupFlags,
    uint32_t        numberOfMembers,
    const uint64_t* members );

OTF2_CallbackCode def_comm(
    void*          userData,
    OTF2_CommRef   self,
    OTF2_StringRef name,
    OTF2_GroupRef  group,
    OTF2_CommRef   parent );

OTF2_CallbackCode def_location_property(
    void*               userData,
    OTF2_LocationRef    location,
    OTF2_StringRef      name,
    OTF2_Type           type,
    OTF2_AttributeValue value );

OTF2_CallbackCode def_calling_context(
    void*                      userData,
    OTF2_CallingContextRef     self,
    OTF2_RegionRef             region,
    OTF2_SourceCodeLocationRef sourceCodeLocation,
    OTF2_CallingContextRef     parent );

OTF2_CallbackCode
def_location_group_property( void*                 userData,
                             OTF2_LocationGroupRef locationGroup,
                             OTF2_StringRef        name,
                             OTF2_Type             type,
                             OTF2_AttributeValue   value );

OTF2_CallbackCode
def_location_property( void*               userData,
                       OTF2_LocationRef    location,
                       OTF2_StringRef      name,
                       OTF2_Type           type,
                       OTF2_AttributeValue value );

/******************************************************************************
 * Event callbacks
 *
 * OTF2 event reader will use the following callbacks when streaming through
 * a trace. This implementation collects time stamps (used for finding compute
 * time between MPI calls) from Enter and Leave callbacks.
 ******************************************************************************
 * Event Behaviors:
 *
 * Most blocking calls will be:
 *    Enter (timestamp)
 *    Leave (timestamp)
 *
 * MPI_Isend
 *    Enter
 *    MPI_Isend (important attributes)
 *    Leave
 *    ...
 *    MPI_Isend_Complete (request ID)
 *
 * MPI_Irecv
 *    Enter
 *    MPI_Irecv_request (request ID)
 *    Leave
 *    ...
 *    MPI_Irecv (important attributes)
 *
 * MPI_Allgather (or any other blocking collective)
 *    Enter
 *    MPI_Collective_Begin
 *    MPI_Collective_End (Important Attributes)
 *    Leave
 *
 * MPI_Wait and MPI_Waitall/some/any do not have dedicated callbacks.
 *    ENTER
 *    MPI_IRECV or MPI_ISEND
 *    LEAVE
 */

OTF2_CallbackCode event_enter(
    OTF2_LocationRef    location,
    OTF2_TimeStamp      time,
    uint64_t            eventPosition,
    void*               userData,
    OTF2_AttributeList* attributes,
    OTF2_RegionRef      region );

OTF2_CallbackCode event_mpi_send(
    OTF2_LocationRef    location,
    OTF2_TimeStamp      time,
    uint64_t            eventPosition,
    void*               userData,
    OTF2_AttributeList* attributes,
    uint32_t            receiver,
    OTF2_CommRef        communicator,
    uint32_t            msgTag,
    uint64_t            msgLength );

OTF2_CallbackCode event_leave(
    OTF2_LocationRef    location,
    OTF2_TimeStamp      time,
    uint64_t            eventPosition,
    void*               userData,
    OTF2_AttributeList* attributes,
    OTF2_RegionRef      region );

OTF2_CallbackCode event_mpi_isend(
    OTF2_LocationRef    location,
    OTF2_TimeStamp      time,
    uint64_t            eventPosition,
    void*               userData,
    OTF2_AttributeList* attributes,
    uint32_t            receiver,
    OTF2_CommRef        communicator,
    uint32_t            msgTag,
    uint64_t            msgLength,
    uint64_t            requestID );

OTF2_CallbackCode event_mpi_isend_complete(
    OTF2_LocationRef    location,
    OTF2_TimeStamp      time,
    uint64_t            eventPosition,
    void*               userData,
    OTF2_AttributeList* attributes,
    uint64_t            requestID );

OTF2_CallbackCode event_mpi_irecv_request(
    OTF2_LocationRef    location,
    OTF2_TimeStamp      time,
    uint64_t            eventPosition,
    void*               userData,
    OTF2_AttributeList* attributes,
    uint64_t            requestID );

OTF2_CallbackCode event_mpi_recv(
    OTF2_LocationRef    location,
    OTF2_TimeStamp      time,
    uint64_t            eventPosition,
    void*               userData,
    OTF2_AttributeList* attributes,
    uint32_t            sender,
    OTF2_CommRef        communicator,
    uint32_t            msgTag,
    uint64_t            msgLength );

OTF2_CallbackCode event_mpi_irecv(
    OTF2_LocationRef    location,
    OTF2_TimeStamp      time,
    uint64_t            eventPosition,
    void*               userData,
    OTF2_AttributeList* attributes,
    uint32_t            sender,
    OTF2_CommRef        communicator,
    uint32_t            msgTag,
    uint64_t            msgLength,
    uint64_t            requestID );

OTF2_CallbackCode event_mpi_request_test(
    OTF2_LocationRef    location,
    OTF2_TimeStamp      time,
    uint64_t            eventPosition,
    void*               userData,
    OTF2_AttributeList* attributes,
    uint64_t            requestID );

OTF2_CallbackCode event_mpi_request_cancelled(
    OTF2_LocationRef    location,
    OTF2_TimeStamp      time,
    uint64_t            eventPosition,
    void*               userData,
    OTF2_AttributeList* attributes,
    uint64_t            requestID );

OTF2_CallbackCode event_mpi_collective_begin(
    OTF2_LocationRef    location,
    OTF2_TimeStamp      time,
    uint64_t            eventPosition,
    void*               userData,
    OTF2_AttributeList* attributes );

OTF2_CallbackCode event_mpi_collective_end(
    OTF2_LocationRef    location,
    OTF2_TimeStamp      time,
    uint64_t            eventPosition,
    void*               userData,
    OTF2_AttributeList* attributes,
    OTF2_CollectiveOp   collectiveOp,
    OTF2_CommRef        communicator,
    uint32_t            root,
    uint64_t            sizeSent,
    uint64_t            sizeReceived );

OTF2_CallbackCode event_parameter_string(
    OTF2_LocationRef    location,
    OTF2_TimeStamp      time,
    uint64_t            eventPosition,
    void*               userData,
    OTF2_AttributeList* attributes,
    OTF2_ParameterRef   parameter,
    OTF2_StringRef      string );

#endif /* CALLBACKS_H_ */
