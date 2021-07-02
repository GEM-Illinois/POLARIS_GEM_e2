/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------
 
  File:        DefaultCameraFactory.cpp

  Description: Implementation of class AVT::VmbAPI::DefaultCameraFactory used to
               create new Camera objects if no user defined camera factory
               class was provided.
               (For internal use only)

-------------------------------------------------------------------------------

  THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF TITLE,
  NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A PARTICULAR  PURPOSE ARE
  DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, 
  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED  
  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR 
  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

=============================================================================*/

#include <VimbaCPP/Source/DefaultCameraFactory.h>

namespace AVT {
namespace VmbAPI {

CameraPtr DefaultCameraFactory::CreateCamera(   const char *pCameraID,
                                                const char *pCameraName,
                                                const char *pCameraModel,
                                                const char *pCameraSerialNumber,
                                                const char *pInterfaceID,
                                                VmbInterfaceType eInterfaceType,
                                                const char * /*pInterfaceName*/,
                                                const char * /*pInterfaceSerialNumber*/,
                                                VmbAccessModeType /*interfacePermittedAccess*/ )
{
    return CameraPtr( new Camera( pCameraID, pCameraName, pCameraModel, pCameraSerialNumber, pInterfaceID, eInterfaceType ));
}

}} // namespace AVT::VmbAPI
